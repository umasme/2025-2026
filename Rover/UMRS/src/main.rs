// UMRS - University of Miami Robotic System
// UM - ASME
// Version 1.0

mod planner;

use socketcan::{CanSocket, Socket, ExtendedId, CanFrame, Frame, Id, EmbeddedFrame};
use std::thread;
use std::time::{Duration, Instant};
use std::io::{self, Write};
use serde::{Deserialize, Serialize};

use crossterm::event::{poll, read, Event, KeyCode};
use crossterm::terminal::{enable_raw_mode, disable_raw_mode, Clear, ClearType};
use crossterm::cursor::MoveTo;
use crossterm::execute;

const WHEEL_RADIUS_M: f32 = 0.13335; 
const TRACK_WIDTH_M: f32 = 1.06625;     
const GEAR_RATIO: f32 = 64.0;        

const FORWARD_SLIP_MULTIPLIER: f32 = 1.0;
const TURN_SCRUB_MULTIPLIER: f32 = 5.0;

const ENABLE_MOTOR_5: bool = true;
const INVERT_MOTOR_5: bool = false;

const ENABLE_MOTOR_1: bool = true;
const ENABLE_MOTOR_2: bool = true; 
const ENABLE_MOTOR_3: bool = true;
const ENABLE_MOTOR_4: bool = true;

const INVERT_MOTOR_1: bool = false;
const INVERT_MOTOR_2: bool = false;
const INVERT_MOTOR_3: bool = false;
const INVERT_MOTOR_4: bool = false;
const MAP_SIZE: usize = 300; 
const MAP_RES: f32 = 0.05;  
const MAP_OFFSET: f32 = 2.0;
const INFLATION_RADIUS_CELLS: isize = 4; 
const OBSTACLE_ACCEPT_RANGE_M: f32 = 1.0;
const OBSTACLE_CLUSTER_RADIUS_M: f32 = 0.3; 
const CONFIDENCE_ON_DETECT: u8 = 80;
const CONFIDENCE_THRESHOLD: u8 = 40;
const CONFIDENCE_DECAY: u8 = 2;
const OBSTACLE_TTL_SECS: f32 = 4.0;        
const MAX_CONSECUTIVE_REPLANS: u8 = 5;
const TARGET_BERM_X: f32 = 6.80;
const TARGET_BERM_Z: f32 = 2.57;
const TARGET_DIG_X: f32 = 1.00;
const TARGET_DIG_Z: f32 = 1.00;
const BOUNDS_X_MIN: f32 = -0.20;
const BOUNDS_X_MAX: f32 = 7.80; 
const BOUNDS_Z_MIN: f32 = -0.20;  
const BOUNDS_Z_MAX: f32 = 3.57;
const CAL_M: f32 = 42.73;
const CAL_B: f32 = 20.0;
const KP_ACTUATOR: f32 = 0.8;
const ACT_MIN_SPEED: i32 = 12;
const DEADBAND_ADC: i32 = 5; 
const KP_DEPOSITION: f32 = 2.8;
const DEPO_MIN_SPEED: i32 = 20;
const DEADBAND_DEPO_CM: f32 = 0.2;
const ACTUATOR_SPEED_SCALE_DOWN: f32 = 0.3;
const ACTUATOR_SPEED_SCALE_UP: f32 = 1.0;
const DEPOSITION_SPEED_SCALE: f32 = 1.0;
const EXCAVATE_TARGET_DEPTH_CM: f32 = 9.0;     
const EXCAVATE_DIG_MOTOR_EFFORT: f32 = -16.0;   
const EXCAVATE_BELT_PAUSE_SECS: f32 = 2.0;     
const EXCAVATE_BELT_INCREMENT_CM: f32 = -5.0;  
const EXCAVATE_BELT_TOTAL_CM: f32 = -25.0;    
const EXCAVATE_BELT_SETTLE_TIMEOUT_SECS: f32 = 2.0;
const EXCAVATE_DRIVE_EFFORT: f32 = 2.0;        
const EXCAVATE_DRIVE_SPEED_SCALE: f32 = 0.05;     
const DUMP_BELT_TRAVEL_CM: f32 = -43.0;          
const DUMP_BELT_SETTLE_TIMEOUT_SECS: f32 = 5.0; 
const DUMP_BACKOUT_DISTANCE_M: f32 = 1.5;    
const DUMP_IN_DISTANCE_M: f32 = 1.0;
const DUMP_IN_EFFORT: f32 = 2.0;     
const DUMP_IN_SPEED_SCALE: f32 = 0.5;    
const DUMP_BACKOUT_EFFORT: f32 = 2.0;     
const DUMP_BACKOUT_SPEED_SCALE: f32 = 0.5;    
const DUMP_OFFSET_Z_PER_CYCLE: f32 = 0.4;      
const WITH_EXCAVATION: bool = false; 
const WITH_DUMPING: bool = false;
const FAKE_EXCAVATE_SECS: f32 = 5.0; 
const FAKE_DUMP_SECS: f32 = 5.0;     
const ARUCO_CHECK_TIMEOUT_SECS: f32 = 10.0;
const ARUCO_TRUST_CLOSE: f32 = 0.85;    
const ARUCO_TRUST_FAR: f32 = 0.30;     
const ARUCO_TRUST_CLOSE_DIST: f32 = 1.0; 
const ARUCO_TRUST_FAR_DIST: f32 = 7.0; 
const BERM_EXCLUSION_X_MIN: f32 = 6.25;   // 6.80 - 0.55
const BERM_EXCLUSION_X_MAX: f32 = 7.35;   // 6.80 + 0.55
const BERM_EXCLUSION_Z_MIN: f32 = 1.72;   // 2.57 - 0.85
const BERM_EXCLUSION_Z_MAX: f32 = 3.42;   // 2.57 + 0.85

#[derive(PartialEq, Debug, Clone, Copy)]
enum RobotState {
    Manual, 
    Localizing, 
    MissionStart,
    Excavate,
    PlanToBerm,
    PlanToDig,
    AutoTurn, 
    AutoDrive, 
    Dump,
    MissionComplete,
    TestTurn,
    TestExcavate,
    TestDump,
    ArucoCheck,
}

#[derive(Serialize, Deserialize, Debug, Default)]
struct Obstacle {
    #[serde(rename = "type")]
    obs_type: String,
    rel_x: f32,
    rel_z: f32,
}

#[derive(Serialize, Deserialize, Debug, Default)]
struct TelemetryData {
    #[serde(default)] aruco_pos: Vec<f32>,
    #[serde(default)] imu_yaw_rate: f32,
    #[serde(default)] vo_dx: f32,
    #[serde(default)] vo_dz: f32,
    #[serde(default)] localization_mode: String,
    #[serde(default)] vo_status: String, 
    #[serde(default)] obstacles: Vec<Obstacle>,
}

#[derive(Serialize)]
struct SimCommand {
    m1: f32, m2: f32, m3: f32, m4: f32,
    x: f32, z: f32, yaw: f32,
}

struct TrackedObstacle {
    x: f32,
    z: f32,
    last_seen: Instant,
}

fn build_velocity_can_id(device_id: u32) -> ExtendedId {
    let device_type = 2 << 24; let manufacturer = 5 << 16;
    let api_class = 1 << 10; let api_index = 2 << 6;  
    let can_id = device_type | manufacturer | api_class | api_index | device_id;
    ExtendedId::new(can_id).expect("Failed to create ExtendedId for Velocity Command")
}

fn send_velocity_command(socket: &CanSocket, device_id: u32, rpm: f32, invert: bool) {
    let mut final_rpm = rpm;
    if invert { final_rpm = -final_rpm; }
    let id = build_velocity_can_id(device_id);
    let rpm_bytes = final_rpm.to_le_bytes();
    let payload: [u8; 8] = [rpm_bytes[0], rpm_bytes[1], rpm_bytes[2], rpm_bytes[3], 0, 0, 0, 1];
    let frame = CanFrame::new(id, &payload).expect("Failed to construct CAN frame");
    let _ = socket.write_frame(&frame);
}

fn send_sabertooth_command(socket: &CanSocket, act_speed: i16, depo_speed: i16) {
    let id = ExtendedId::new(0x100001).expect("Failed to create ExtendedId for Sabertooth");
    let act_bytes = act_speed.to_le_bytes();
    let depo_bytes = depo_speed.to_le_bytes();
    let payload: [u8; 8] = [
        act_bytes[0], act_bytes[1],
        depo_bytes[0], depo_bytes[1], 
        0, 0, 0, 0
    ];
    let frame = CanFrame::new(id, &payload).expect("Failed to construct CAN frame");
    let _ = socket.write_frame(&frame);
}

const PATH_CHECK_CORRIDOR_CELLS: isize = 5;

fn is_path_blocked(map: &[[u8; MAP_SIZE]; MAP_SIZE], start_x: f32, start_z: f32, goal_x: f32, goal_z: f32) -> bool {
    let mut x0 = ((start_x + MAP_OFFSET) / MAP_RES).round() as isize;
    let mut z0 = ((start_z + MAP_OFFSET) / MAP_RES).round() as isize;
    let x1 = ((goal_x + MAP_OFFSET) / MAP_RES).round() as isize;
    let z1 = ((goal_z + MAP_OFFSET) / MAP_RES).round() as isize;
    let dx = (x1 - x0).abs(); let dz = (z1 - z0).abs();
    let sx = if x0 < x1 { 1 } else { -1 }; let sz = if z0 < z1 { 1 } else { -1 };
    let mut err = dx - dz;

    while x0 != x1 || z0 != z1 {
        for cx in -PATH_CHECK_CORRIDOR_CELLS..=PATH_CHECK_CORRIDOR_CELLS {
            for cz in -PATH_CHECK_CORRIDOR_CELLS..=PATH_CHECK_CORRIDOR_CELLS {
                let check_x = x0 + cx;
                let check_z = z0 + cz;
                if check_x >= 0 && check_x < MAP_SIZE as isize && check_z >= 0 && check_z < MAP_SIZE as isize {
                    if map[check_x as usize][check_z as usize] >= CONFIDENCE_THRESHOLD { return true; }
                }
            }
        }
        let e2 = 2 * err;
        if e2 > -dz { err -= dz; x0 += sx; }
        if e2 < dx { err += dx; z0 += sz; }
    }
    false
}

fn wrap_pi(mut a: f32) -> f32 {
    while a > std::f32::consts::PI {
        a -= 2.0 * std::f32::consts::PI;
    }
    while a < -std::f32::consts::PI {
        a += 2.0 * std::f32::consts::PI;
    }
    a
}

fn wrap_deg(mut a: f32) -> f32 {
    while a > 180.0 {
        a -= 360.0;
    }
    while a < -180.0 {
        a += 360.0;
    }
    a
}

fn distance_to_boundary(x: f32, z: f32, dx: f32, dz: f32,
                        bx_min: f32, bx_max: f32, bz_min: f32, bz_max: f32) -> f32 {
    let mut t_min = f32::MAX;
    if dx >  0.001 { t_min = t_min.min((bx_max - x) / dx); }
    if dx < -0.001 { t_min = t_min.min((bx_min - x) / dx); }
    if dz >  0.001 { t_min = t_min.min((bz_max - z) / dz); }
    if dz < -0.001 { t_min = t_min.min((bz_min - z) / dz); }
    t_min.max(0.0)
}

fn main() {
    let rx_socket = CanSocket::open("can0").expect("CRITICAL: Failed to open CAN rx_socket");
    rx_socket.set_nonblocking(true).expect("Failed to set CAN socket to non-blocking");
    let tx_socket = CanSocket::open("can0").expect("CRITICAL: Failed to open CAN tx_socket");

    send_velocity_command(&tx_socket, 1, 0.0, false);
    send_velocity_command(&tx_socket, 2, 0.0, false);
    send_velocity_command(&tx_socket, 3, 0.0, false);
    send_velocity_command(&tx_socket, 4, 0.0, false);
    send_velocity_command(&tx_socket, 5, 0.0, false);
    send_sabertooth_command(&tx_socket, 0, 0);
    thread::sleep(Duration::from_millis(50));

    let heartbeat_rx_socket = CanSocket::open("can0").expect("Failed to open heartbeat socket");
    heartbeat_rx_socket.set_nonblocking(true).expect("Failed to set heartbeat socket to non-blocking");

    let context = zmq::Context::new();
    let subscriber = context.socket(zmq::SUB).unwrap();
    subscriber.connect("tcp://localhost:5555").expect("Failed to connect to ZMQ");
    subscriber.set_subscribe(b"").expect("Failed to subscribe to ZMQ");

    let publisher = context.socket(zmq::PUB).unwrap();
    publisher.bind("tcp://*:5556").expect("Failed to bind Sim Publisher");
    let mut print_timer = Instant::now();
    let mut action_timer = Instant::now();
    let mut current_state = RobotState::Manual;
    let mut cycle_count: u8 = 0;
    let mut target_x: f32 = 0.0;
    let mut target_z: f32 = 0.0;
    let mut waypoints: Vec<(f32, f32)> = Vec::new();
    let mut path_status = String::from("IDLE");
    let mut global_x: f32 = 0.50;
    let mut global_z: f32 = 0.50;
    let mut logical_yaw: f32 = std::f32::consts::PI / 2.0;
    let mut loc_mode = String::from("WAITING");
    let mut cfg_target_berm_x: f32 = TARGET_BERM_X;
    let mut cfg_target_berm_z: f32 = TARGET_BERM_Z;
    let mut cfg_target_dig_x: f32 = TARGET_DIG_X;
    let mut cfg_target_dig_z: f32 = TARGET_DIG_Z;
    let mut cfg_bounds_x_min: f32 = BOUNDS_X_MIN;
    let mut cfg_bounds_x_max: f32 = BOUNDS_X_MAX;
    let mut cfg_bounds_z_min: f32 = BOUNDS_Z_MIN;
    let mut cfg_bounds_z_max: f32 = BOUNDS_Z_MAX;
    let mut cfg_berm_excl_x_min: f32 = BERM_EXCLUSION_X_MIN;
    let mut cfg_berm_excl_x_max: f32 = BERM_EXCLUSION_X_MAX;
    let mut cfg_berm_excl_z_min: f32 = BERM_EXCLUSION_Z_MIN;
    let mut cfg_berm_excl_z_max: f32 = BERM_EXCLUSION_Z_MAX;
    let mut turn_in_progress = false;
    let mut turn_target_pos: f32 = 0.0;
    let mut arena_map = [[0u8; MAP_SIZE]; MAP_SIZE];
    let mut tracked_obstacles: Vec<TrackedObstacle> = Vec::new();
    let mut show_map = false; 
    let mut consecutive_replans: u8 = 0;
    let mut berm_exclusion_active: bool = false;
    let mut base_speed: f32 = 80.0;
    let mut cmd_m1: f32 = 0.0; 
    let mut cmd_m2: f32 = 0.0; 
    let mut cmd_m3: f32 = 0.0; 
    let mut cmd_m4: f32 = 0.0;
    let mut cmd_m5: f32 = 0.0;
    let mut act_speed: i16 = 0; 
    let mut depo_speed: i16 = 0; 
    let mut actual_rpm_m1: f32 = 0.0; 
    let mut actual_rpm_m2: f32 = 0.0; 
    let mut actual_rpm_m3: f32 = 0.0; 
    let mut actual_rpm_m4: f32 = 0.0;
    let mut actual_rpm_m5: f32 = 0.0;
    let mut actual_pos_m1: f32 = 0.0;
    let mut imu_yaw_deg: f32 = 0.0;
    let mut imu_pitch_deg: f32 = 0.0;
    let mut imu_roll_deg: f32 = 0.0;
    let mut current_adc_reading: i32 = 0;
    let mut current_depo_cm: f32 = 0.0;
    let mut initial_depo_offset: Option<f32> = None;
    let mut target_depth_cm: Option<f32> = None;
    let mut target_depo_cm: Option<f32> = None;
    let mut error_ticks: i32 = 0;
    let mut error_depo: f32 = 0.0;
    let mut excavate_phase: u8 = 0;
    let mut excavate_belt_laps_done: u32 = 0;
    let excavate_belt_total_laps: u32 = (EXCAVATE_BELT_TOTAL_CM / EXCAVATE_BELT_INCREMENT_CM).round() as u32;
    let mut excavate_belt_paused: bool = false;
    let mut dump_phase: u8 = 0;
    let mut dump_backout_start_x: f32 = 0.0;
    let mut dump_backout_start_z: f32 = 0.0;
    let mut dump_backout_forward: bool = false; 
    let mut aruco_check_phase: u8 = 0;
    let mut aruco_check_pre_yaw: f32 = 0.0;    
    let mut aruco_check_resume_state: u8 = 0;  
    let mut aruco_check_got_fix: bool = false;  
    let mut aruco_check_timer = Instant::now(); 
    let mut last_kinematics_time = Instant::now();
    let mut last_imu_yaw_deg: f32 = imu_yaw_deg;
    let mut imu_yaw_initialized: bool = false;

    enable_raw_mode().expect("Failed to enable raw mode");
    execute!(io::stdout(), Clear(ClearType::All)).unwrap();

    thread::spawn(move || {
        let heartbeat_id = ExtendedId::new(0x02052C80).unwrap();
        let heartbeat_frame = CanFrame::new(heartbeat_id, &[0xFF; 8]).unwrap();
        let hb_tx_socket = CanSocket::open("can0").unwrap();
        loop {
            while let Ok(_) = heartbeat_rx_socket.read_frame() {} 
            let _ = hb_tx_socket.write_frame(&heartbeat_frame);
            thread::sleep(Duration::from_millis(20));
        }
    });

    loop {
        if poll(Duration::from_millis(0)).unwrap() {
            if let Event::Key(key_event) = read().unwrap() {
                match key_event.code {
                    KeyCode::Char('m') | KeyCode::Char('M') | KeyCode::Char(' ') => {
                        current_state = RobotState::Manual;
                        cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0; cmd_m5 = 0.0; 
                        act_speed = 0; 
                        depo_speed = 0; 
                        
                        target_depth_cm = Some((current_adc_reading as f32 - CAL_B) / CAL_M);
                        target_depo_cm = Some(current_depo_cm);
                        
                        path_status = String::from("ABORTED");
                        consecutive_replans = 0;
                        berm_exclusion_active = false;
                    },
                    KeyCode::Char('l') | KeyCode::Char('L') => {
                        current_state = RobotState::Localizing;
                        cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0; cmd_m5 = 0.0;
                    },
                    KeyCode::Char('p') | KeyCode::Char('P') => { show_map = !show_map; execute!(io::stdout(), Clear(ClearType::All)).unwrap(); },
                    KeyCode::Char('g') | KeyCode::Char('G') => {
                        if current_state == RobotState::Localizing || current_state == RobotState::Manual {
                            disable_raw_mode().expect("Failed to disable raw mode");
                            execute!(io::stdout(), MoveTo(0, 0), Clear(ClearType::All)).unwrap();

                            println!("=============================================");
                            println!("       ARENA CONFIGURATION — PRESS 1 OR 2    ");
                            println!("=============================================");
                            println!("  Option 1 (Default — ArUco at origin):");
                            println!("    Berm:  ({:.2}, {:.2})", TARGET_BERM_X, TARGET_BERM_Z);
                            println!("    Dig:   ({:.2}, {:.2})", TARGET_DIG_X, TARGET_DIG_Z);
                            println!("    Start: (0.50, 0.50)  ArUco (0, 0)");
                            println!("");
                            println!("  Option 2 (Secondary — ArUco at Z=6):");
                            println!("    Berm:  (6.80, 1.57)");
                            println!("    Dig:   (1.00, 6.00)");
                            println!("    Start: (0.50, 5.50)  ArUco (0, 6)");
                            println!("=============================================");
                            print!("Select (1/2): ");
                            io::stdout().flush().unwrap();

                            let mut input = String::new();
                            let mut selected = 1u8;
                            if io::stdin().read_line(&mut input).is_ok() {
                                if input.trim() == "2" { selected = 2; }
                            }

                            if selected == 2 {
                                cfg_target_berm_x = 6.80;
                                cfg_target_berm_z = 1.57;
                                cfg_target_dig_x = 1.00;
                                cfg_target_dig_z = 6.00;
                                global_x = 0.50;
                                global_z = 5.50;
                                logical_yaw = std::f32::consts::PI / 2.0;
                                cfg_bounds_x_min = BOUNDS_X_MIN;
                                cfg_bounds_x_max = BOUNDS_X_MAX;
                                cfg_bounds_z_min = -0.20;
                                cfg_bounds_z_max = 7.00;
                                cfg_berm_excl_x_min = BERM_EXCLUSION_X_MIN;
                                cfg_berm_excl_x_max = BERM_EXCLUSION_X_MAX;
                                cfg_berm_excl_z_min = 0.72;
                                cfg_berm_excl_z_max = 2.42;
                            } else {
                                cfg_target_berm_x = TARGET_BERM_X;
                                cfg_target_berm_z = TARGET_BERM_Z;
                                cfg_target_dig_x = TARGET_DIG_X;
                                cfg_target_dig_z = TARGET_DIG_Z;
                                global_x = 0.50;
                                global_z = 0.50;
                                logical_yaw = std::f32::consts::PI / 2.0;
                                cfg_bounds_x_min = BOUNDS_X_MIN;
                                cfg_bounds_x_max = BOUNDS_X_MAX;
                                cfg_bounds_z_min = BOUNDS_Z_MIN;
                                cfg_bounds_z_max = BOUNDS_Z_MAX;
                                cfg_berm_excl_x_min = BERM_EXCLUSION_X_MIN;
                                cfg_berm_excl_x_max = BERM_EXCLUSION_X_MAX;
                                cfg_berm_excl_z_min = BERM_EXCLUSION_Z_MIN;
                                cfg_berm_excl_z_max = BERM_EXCLUSION_Z_MAX;
                            }

                            enable_raw_mode().expect("Failed to enable raw mode");
                            execute!(io::stdout(), Clear(ClearType::All)).unwrap();

                            current_state = RobotState::MissionStart;
                            consecutive_replans = 0;
                        }
                    },
                    KeyCode::Esc | KeyCode::Char('q') | KeyCode::Char('Q') => { break; },
                    
                    KeyCode::Char('1') => {
                        if current_state == RobotState::Manual || current_state == RobotState::Localizing {
                            turn_target_pos = logical_yaw + (std::f32::consts::PI / 2.0);
                            while turn_target_pos > std::f32::consts::PI { turn_target_pos -= 2.0 * std::f32::consts::PI; }
                            while turn_target_pos < -std::f32::consts::PI { turn_target_pos += 2.0 * std::f32::consts::PI; }
                            current_state = RobotState::TestTurn;
                            path_status = String::from("TEST TURN 90 RIGHT");
                        }
                    },
                    KeyCode::Char('2') => {
                        if current_state == RobotState::Manual || current_state == RobotState::Localizing {
                            turn_target_pos = logical_yaw - (std::f32::consts::PI / 2.0); 
                            while turn_target_pos > std::f32::consts::PI { turn_target_pos -= 2.0 * std::f32::consts::PI; }
                            while turn_target_pos < -std::f32::consts::PI { turn_target_pos += 2.0 * std::f32::consts::PI; }
                            current_state = RobotState::TestTurn;
                            path_status = String::from("TEST TURN 90 LEFT");
                        }
                    },
                    KeyCode::Char('3') => {
                        if current_state == RobotState::Manual || current_state == RobotState::Localizing {
                            turn_target_pos = logical_yaw + std::f32::consts::PI; 
                            while turn_target_pos > std::f32::consts::PI { turn_target_pos -= 2.0 * std::f32::consts::PI; }
                            while turn_target_pos < -std::f32::consts::PI { turn_target_pos += 2.0 * std::f32::consts::PI; }
                            current_state = RobotState::TestTurn;
                            path_status = String::from("TEST TURN 180 DEG");
                        }
                    },

                    KeyCode::Char('7') => {
                        if current_state == RobotState::Manual || current_state == RobotState::Localizing {
                            excavate_phase = 0;
                            excavate_belt_laps_done = 0;
                            excavate_belt_paused = false;
                            current_state = RobotState::TestExcavate;
                            action_timer = Instant::now();
                            path_status = String::from("TEST EXCAVATION STARTED");
                        }
                    },

                    KeyCode::Char('9') => {
                        if current_state == RobotState::Manual || current_state == RobotState::Localizing {
                            dump_phase = 0;
                            dump_backout_start_x = global_x;
                            dump_backout_start_z = global_z;
                            current_state = RobotState::TestDump;
                            action_timer = Instant::now();
                            path_status = String::from("TEST DUMP STARTED");
                        }
                    },

                    KeyCode::Char('o') | KeyCode::Char('O') | KeyCode::Char('k') | KeyCode::Char('K') => {
                        disable_raw_mode().expect("Failed to disable raw mode");
                        execute!(io::stdout(), MoveTo(0, 0), Clear(ClearType::All)).unwrap();
                        
                        let current_cm = (current_adc_reading as f32 - CAL_B) / CAL_M;
                        let tgt_display = target_depth_cm.unwrap_or(current_cm);

                        println!("=====================================");
                        println!("    ACTUATOR DEPTH CONTROL MENU      ");
                        println!("=====================================");
                        println!("Current Actual Depth : {:.2} cm", current_cm);
                        println!("Current Target Depth : {:.2} cm", tgt_display);
                        print!("\nEnter target (+/- for relative, else absolute) (cm): ");
                        io::stdout().flush().unwrap();
                        
                        let mut input = String::new();
                        if io::stdin().read_line(&mut input).is_ok() {
                            let trimmed = input.trim();
                            if trimmed.starts_with('+') || trimmed.starts_with('-') {
                                if let Ok(rel) = trimmed.parse::<f32>() {
                                    target_depth_cm = Some(current_cm + rel);
                                }
                            } else {
                                if let Ok(abs) = trimmed.parse::<f32>() {
                                    target_depth_cm = Some(abs);
                                }
                            }
                        }
                        
                        enable_raw_mode().expect("Failed to enable raw mode");
                        execute!(io::stdout(), Clear(ClearType::All)).unwrap();
                    },

                    KeyCode::Char('z') | KeyCode::Char('Z') | KeyCode::Char('y') | KeyCode::Char('Y') => {
                        disable_raw_mode().expect("Failed to disable raw mode");
                        execute!(io::stdout(), MoveTo(0, 0), Clear(ClearType::All)).unwrap();
                        
                        let tgt_display = target_depo_cm.unwrap_or(current_depo_cm);

                        println!("=====================================");
                        println!("    DEPOSITION CONTROL MENU          ");
                        println!("=====================================");
                        println!("Current Depo Pos : {:.2} cm", current_depo_cm);
                        println!("Current Target   : {:.2} cm", tgt_display);
                        print!("\nEnter target (+/- for relative, else absolute) (cm): ");
                        io::stdout().flush().unwrap();
                        
                        let mut input = String::new();
                        if io::stdin().read_line(&mut input).is_ok() {
                            let trimmed = input.trim();
                            if trimmed.starts_with('+') || trimmed.starts_with('-') {
                                if let Ok(rel) = trimmed.parse::<f32>() {
                                    target_depo_cm = Some(current_depo_cm + rel);
                                }
                            } else {
                                if let Ok(abs) = trimmed.parse::<f32>() {
                                    target_depo_cm = Some(abs);
                                }
                            }
                        }
                        
                        enable_raw_mode().expect("Failed to enable raw mode");
                        execute!(io::stdout(), Clear(ClearType::All)).unwrap();
                    },

                    _ => {
                        if current_state == RobotState::Manual || current_state == RobotState::Localizing {
                            match key_event.code {
                                KeyCode::Char('w') | KeyCode::Char('W') => { cmd_m1 = -2.0; cmd_m2 = -2.0; cmd_m3 = 2.0; cmd_m4 = 2.0; },
                                KeyCode::Char('s') | KeyCode::Char('S') => { cmd_m1 = 2.0; cmd_m2 = 2.0; cmd_m3 = -2.0; cmd_m4 = -2.0; },
                                KeyCode::Char('a') | KeyCode::Char('A') => { cmd_m1 = 2.0; cmd_m2 = 2.0; cmd_m3 = 2.0; cmd_m4 = 2.0; }, 
                                KeyCode::Char('d') | KeyCode::Char('D') => { cmd_m1 = -2.0; cmd_m2 = -2.0; cmd_m3 = -2.0; cmd_m4 = -2.0; }, 

                                KeyCode::Char('x') | KeyCode::Char('X') => { 
                                    cmd_m5 = 0.0; 
                                    depo_speed = 0; 
                                    target_depo_cm = Some(current_depo_cm); 
                                }, 

                                KeyCode::Char('v') | KeyCode::Char('V') => {
                                    cmd_m5 = -20.0;
                                    depo_speed = 0;
                                    target_depo_cm = Some(current_depo_cm);
                                },
                                
                                KeyCode::Up => { base_speed += 50.0; },
                                KeyCode::Down => { if base_speed > 50.0 { base_speed -= 50.0; } },
                                _ => {}
                            }
                        }
                    }
                }
            }
        }

        while let Ok(frame) = rx_socket.read_frame() {
            let id = frame.id();
            if let Id::Extended(ext_id) = id {
                let can_id = ext_id.as_raw(); 
                
                if can_id == 0x200001 {
                    let payload = frame.data();
                    if payload.len() >= 2 {
                        let low_byte = payload[0] as u16;
                        let high_byte = payload[1] as u16;
                        current_adc_reading = ((high_byte << 8) | low_byte) as i32;
                    }
                    if payload.len() >= 6 {
                        let mut float_bytes = [0u8; 4]; 
                        float_bytes.copy_from_slice(&payload[2..6]);
                        let raw_depo = f32::from_le_bytes(float_bytes);
                        let offset = *initial_depo_offset.get_or_insert(raw_depo);
                        current_depo_cm = raw_depo - offset;
                    }
                } else if can_id == 0x200002 {
                    let payload = frame.data();
                    if payload.len() >= 6 {
                        let mut yaw_bytes = [0u8; 2];
                        let mut pitch_bytes = [0u8; 2];
                        let mut roll_bytes = [0u8; 2];
                        yaw_bytes.copy_from_slice(&payload[0..2]);
                        pitch_bytes.copy_from_slice(&payload[2..4]);
                        roll_bytes.copy_from_slice(&payload[4..6]);
                        imu_yaw_deg = i16::from_le_bytes(yaw_bytes) as f32 / 100.0;
                        imu_pitch_deg = i16::from_le_bytes(pitch_bytes) as f32 / 100.0;
                        imu_roll_deg = i16::from_le_bytes(roll_bytes) as f32 / 100.0;
                    }
                } else {
                    let device_id = can_id & 0x3F; let api_index = (can_id >> 6) & 0xF; 
                    let api_class = (can_id >> 10) & 0x3F; let manufacturer = (can_id >> 16) & 0xFF;

                    if manufacturer == 5 && api_class == 6 {
                        let payload = frame.data();
                        if payload.len() >= 4 {
                            if api_index == 1 {
                                let mut rpm_bytes = [0u8; 4]; rpm_bytes.copy_from_slice(&payload[0..4]);
                                let current_rpm = f32::from_le_bytes(rpm_bytes);
                                match device_id { 
                                    1 => actual_rpm_m1 = current_rpm, 2 => actual_rpm_m2 = current_rpm, 
                                    3 => actual_rpm_m3 = current_rpm, 4 => actual_rpm_m4 = current_rpm, 
                                    5 => actual_rpm_m5 = current_rpm, _ => {} 
                                }
                            } else if api_index == 2 && device_id == 1 {
                                let mut pos_bytes = [0u8; 4]; pos_bytes.copy_from_slice(&payload[0..4]);
                                actual_pos_m1 = f32::from_le_bytes(pos_bytes);
                            }
                        }
                    }
                }
            }
        }

        if current_adc_reading != 0 {
            let tgt_depth = target_depth_cm.unwrap_or_else(|| (current_adc_reading as f32 - CAL_B) / CAL_M);
            target_depth_cm = Some(tgt_depth); 
            
            let target_adc = (tgt_depth * CAL_M + CAL_B) as i32;
            error_ticks = target_adc - current_adc_reading;
            
            if error_ticks.abs() < DEADBAND_ADC {
                act_speed = 0;
            } else {
                let mut speed = (error_ticks as f32 * KP_ACTUATOR) as i32;
                if speed > 0 { speed += ACT_MIN_SPEED; }
                else if speed < 0 { speed -= ACT_MIN_SPEED; }
                let act_scale = if speed >= 0 { ACTUATOR_SPEED_SCALE_DOWN } else { ACTUATOR_SPEED_SCALE_UP };
                let scaled_speed = (speed as f32 * act_scale) as i32;
                act_speed = scaled_speed.clamp(-127, 127) as i16;
            }

            let tgt_depo = target_depo_cm.unwrap_or(current_depo_cm);
            target_depo_cm = Some(tgt_depo); 
            
            error_depo = tgt_depo - current_depo_cm;
            
            if error_depo.abs() < DEADBAND_DEPO_CM {
                depo_speed = 0;
            } else {
                let mut speed = (error_depo * KP_DEPOSITION) as i32;
                if speed > 0 { speed += DEPO_MIN_SPEED; }
                else if speed < 0 { speed -= DEPO_MIN_SPEED; }
                let scaled_speed = (speed as f32 * DEPOSITION_SPEED_SCALE) as i32;
                depo_speed = scaled_speed.clamp(-127, 127) as i16;
            }
        } else {
            act_speed = 0;
            depo_speed = 0;
        }

        while let Ok(Ok(msg)) = subscriber.recv_string(zmq::DONTWAIT) {
            if let Ok(data) = serde_json::from_str::<TelemetryData>(&msg) {
                if data.localization_mode == "ARUCO_LOCKED" && data.aruco_pos.len() >= 2 {
                    let aruco_x = data.aruco_pos[0];
                    let aruco_z = data.aruco_pos[1];
                    let dist_to_marker = f32::sqrt(aruco_x * aruco_x + aruco_z * aruco_z);
                    let trust = if dist_to_marker <= ARUCO_TRUST_CLOSE_DIST {
                        ARUCO_TRUST_CLOSE
                    } else if dist_to_marker >= ARUCO_TRUST_FAR_DIST {
                        ARUCO_TRUST_FAR
                    } else {
                        let t = (dist_to_marker - ARUCO_TRUST_CLOSE_DIST) 
                              / (ARUCO_TRUST_FAR_DIST - ARUCO_TRUST_CLOSE_DIST);
                        ARUCO_TRUST_CLOSE + t * (ARUCO_TRUST_FAR - ARUCO_TRUST_CLOSE)
                    };

                    let at_natural_stop = match current_state {
                        RobotState::Localizing => true,
                        RobotState::ArucoCheck => true,
                        _ => false,
                    };

                    let moving = cmd_m1 != 0.0 || cmd_m2 != 0.0 || cmd_m3 != 0.0 || cmd_m4 != 0.0;

                    let pos_alpha = if at_natural_stop {
                        trust.min(0.60)
                    } else if moving {
                        trust.min(0.08)
                    } else {
                        trust.min(0.25)
                    };

                    let yaw_alpha = if at_natural_stop {
                        trust.min(0.70)
                    } else if moving {
                        trust.min(0.10)
                    } else {
                        trust.min(0.30)
                    };

                    let pos_err = f32::sqrt((aruco_x - global_x).powi(2) + (aruco_z - global_z).powi(2));
                    let yaw_err = if data.aruco_pos.len() == 3 {
                        wrap_pi(data.aruco_pos[2] - logical_yaw).abs()
                    } else {
                        0.0
                    };

                    let aruco_gate_ok = at_natural_stop || (pos_err < 1.0 && yaw_err < 1.2);

                    if aruco_gate_ok {
                        global_x = global_x * (1.0 - pos_alpha) + aruco_x * pos_alpha;
                        global_z = global_z * (1.0 - pos_alpha) + aruco_z * pos_alpha;

                        if data.aruco_pos.len() == 3 {
                            let yaw_correction = wrap_pi(data.aruco_pos[2] - logical_yaw);
                            logical_yaw = wrap_pi(logical_yaw + yaw_alpha * yaw_correction);
                        }

                        if current_state == RobotState::ArucoCheck && aruco_check_phase == 2 {
                            aruco_check_got_fix = true;
                        }
                    }
                    loc_mode = format!("ARUCO(d={:.1}m,t={:.0}%)", dist_to_marker, trust * 100.0);
                } else {
                    loc_mode = String::from("BLIND (ENC)");
                }

                for obs in &data.obstacles {
                    let obs_dist = f32::sqrt(obs.rel_x * obs.rel_x + obs.rel_z * obs.rel_z);
                    if obs_dist > OBSTACLE_ACCEPT_RANGE_M {
                        continue;
                    }

                    let obs_global_x = global_x + (obs.rel_z * logical_yaw.sin()) + (obs.rel_x * logical_yaw.cos());
                    let obs_global_z = global_z + (obs.rel_z * logical_yaw.cos()) - (obs.rel_x * logical_yaw.sin());

                    let mut found = false;
                    for tracked in &mut tracked_obstacles {
                        let dist = f32::sqrt((tracked.x - obs_global_x).powi(2) + (tracked.z - obs_global_z).powi(2));
                        if dist < OBSTACLE_CLUSTER_RADIUS_M {
                            tracked.x = tracked.x * 0.8 + obs_global_x * 0.2; 
                            tracked.z = tracked.z * 0.8 + obs_global_z * 0.2;
                            tracked.last_seen = Instant::now();
                            found = true;
                            break;
                        }
                    }

                    if !found {
                        tracked_obstacles.push(TrackedObstacle {
                            x: obs_global_x, z: obs_global_z, last_seen: Instant::now(),
                        });
                    }
                }
            }
        }

        tracked_obstacles.retain(|obs| obs.last_seen.elapsed().as_secs_f32() < OBSTACLE_TTL_SECS);

        for x in 0..MAP_SIZE { 
            for z in 0..MAP_SIZE { 
                arena_map[x][z] = arena_map[x][z].saturating_sub(CONFIDENCE_DECAY);
            } 
        }

        for obs in &tracked_obstacles {

            let dx_to_rover = obs.x - global_x;
            let dz_to_rover = obs.z - global_z;
            let dist_to_rover = f32::sqrt(dx_to_rover * dx_to_rover + dz_to_rover * dz_to_rover);
            if dist_to_rover > OBSTACLE_ACCEPT_RANGE_M + 0.5 {
                continue; 
            }

            if berm_exclusion_active 
                && obs.x >= cfg_berm_excl_x_min && obs.x <= cfg_berm_excl_x_max
                && obs.z >= cfg_berm_excl_z_min && obs.z <= cfg_berm_excl_z_max
            {
                continue; 
            }

            let center_x = ((obs.x + MAP_OFFSET) / MAP_RES).round() as isize;
            let center_z = ((obs.z + MAP_OFFSET) / MAP_RES).round() as isize;

            for ix in -INFLATION_RADIUS_CELLS..=INFLATION_RADIUS_CELLS {
                for iz in -INFLATION_RADIUS_CELLS..=INFLATION_RADIUS_CELLS {
                    if ix * ix + iz * iz <= INFLATION_RADIUS_CELLS * INFLATION_RADIUS_CELLS {
                        let tx = center_x + ix; let tz = center_z + iz;
                        if tx >= 0 && tx < MAP_SIZE as isize && tz >= 0 && tz < MAP_SIZE as isize {
                            let cell = &mut arena_map[tx as usize][tz as usize];
                            if *cell < CONFIDENCE_ON_DETECT {
                                *cell = CONFIDENCE_ON_DETECT;
                            }
                        }
                    }
                }
            }
        }

        {
            let grid_x_min = ((cfg_bounds_x_min + MAP_OFFSET) / MAP_RES).round() as isize;
            let grid_x_max = ((cfg_bounds_x_max + MAP_OFFSET) / MAP_RES).round() as isize;
            let grid_z_min = ((cfg_bounds_z_min + MAP_OFFSET) / MAP_RES).round() as isize;
            let grid_z_max = ((cfg_bounds_z_max + MAP_OFFSET) / MAP_RES).round() as isize;

            for x in 0..MAP_SIZE {
                for z in 0..MAP_SIZE {
                    let ix = x as isize;
                    let iz = z as isize;
                    if ix <= grid_x_min || ix >= grid_x_max || iz <= grid_z_min || iz >= grid_z_max {
                        arena_map[x][z] = 255;
                    }
                }
            }
        }

        let rpm_to_mps = (2.0 * std::f32::consts::PI * WHEEL_RADIUS_M) / 60.0;
        let effective_rpm_left_raw = (actual_rpm_m1 + actual_rpm_m2) / 2.0;
        let effective_rpm_right_raw = (actual_rpm_m3 + actual_rpm_m4) / 2.0; 
        let v_left_raw = (effective_rpm_left_raw / GEAR_RATIO) * rpm_to_mps; 
        let v_right_raw = (effective_rpm_right_raw / GEAR_RATIO) * rpm_to_mps;
        let v_left_true = -v_left_raw; 
        let v_right_true = v_right_raw; 

        let mut encoder_v_forward = (v_left_true + v_right_true) / 2.0; 
        let encoder_omega = (v_left_true - v_right_true) / TRACK_WIDTH_M;
        if cmd_m1 != 0.0 && cmd_m1 == cmd_m3 { encoder_v_forward = 0.0; }

        let dt_kinematics = last_kinematics_time.elapsed().as_secs_f32();
        last_kinematics_time = Instant::now();

        if dt_kinematics < 0.2 {
            // IMU yaw is the continuous heading source.
            // Encoders only provide forward distance.
            if !imu_yaw_initialized {
                last_imu_yaw_deg = imu_yaw_deg;
                imu_yaw_initialized = true;
            } else {
                let imu_delta_deg = wrap_deg(imu_yaw_deg - last_imu_yaw_deg);
                let imu_delta_rad = imu_delta_deg * (std::f32::consts::PI / 180.0);
                logical_yaw = wrap_pi(logical_yaw + imu_delta_rad);
                last_imu_yaw_deg = imu_yaw_deg;
            }

            let delta_d = encoder_v_forward * dt_kinematics * FORWARD_SLIP_MULTIPLIER;
            global_x += delta_d * logical_yaw.sin();
            global_z += delta_d * logical_yaw.cos();
        }

        if current_state != RobotState::Manual 
            && current_state != RobotState::Localizing 
            && current_state != RobotState::MissionComplete 
            && current_state != RobotState::ArucoCheck
            && current_state != RobotState::TestExcavate
            && current_state != RobotState::TestDump
            && current_state != RobotState::TestTurn
        {
            if global_x < cfg_bounds_x_min || global_x > cfg_bounds_x_max 
                || global_z < cfg_bounds_z_min || global_z > cfg_bounds_z_max 
            {
                cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0; cmd_m5 = 0.0;
                path_status = format!(
                    "OUT OF BOUNDS @ ({:.2},{:.2}) — HALTING", global_x, global_z
                );
                current_state = RobotState::Manual;
                consecutive_replans = 0;
                berm_exclusion_active = false;
                waypoints.clear();
            }
        }

        match current_state {
            RobotState::Manual | RobotState::Localizing | RobotState::MissionComplete => {},

            RobotState::TestTurn => {
                let mut angle_diff = turn_target_pos - logical_yaw;
                while angle_diff > std::f32::consts::PI { angle_diff -= 2.0 * std::f32::consts::PI; }
                while angle_diff < -std::f32::consts::PI { angle_diff += 2.0 * std::f32::consts::PI; }

                if angle_diff.abs() <= 0.15 { 
                    cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                    current_state = RobotState::Manual;
                    path_status = String::from("TEST TURN COMPLETE");
                } else {
                    let constant_turn_rpm = 160.0;
                    let turn_effort = (constant_turn_rpm * angle_diff.signum()) / base_speed; 
                    cmd_m1 = -turn_effort; cmd_m2 = -turn_effort; cmd_m3 = -turn_effort; cmd_m4 = -turn_effort;     
                }
            },

            RobotState::MissionStart => {
                cycle_count = 0;
                consecutive_replans = 0;
                berm_exclusion_active = false;
                path_status = String::from("MISSION INITIATED - CYCLE 1");
                action_timer = Instant::now();
                excavate_phase = 0;
                excavate_belt_laps_done = 0;
                excavate_belt_paused = false;
                current_state = RobotState::Excavate;
            },

            RobotState::Excavate => {
                cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;

                if !WITH_EXCAVATION {
                    let elapsed = action_timer.elapsed().as_secs_f32();
                    path_status = format!("[CYCLE {}] FAKE EXCAVATE: {:.1}s / {:.1}s...", 
                        cycle_count + 1, elapsed, FAKE_EXCAVATE_SECS);
                    cmd_m5 = 0.0;
                    let creep = EXCAVATE_DRIVE_EFFORT * EXCAVATE_DRIVE_SPEED_SCALE;
                    cmd_m1 = -creep; cmd_m2 = -creep; cmd_m3 = creep; cmd_m4 = creep;

                    if elapsed >= FAKE_EXCAVATE_SECS {
                        cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                        path_status = format!("[CYCLE {}] FAKE EXCAVATION COMPLETE", cycle_count + 1);
                        target_x = cfg_target_berm_x;
                        target_z = cfg_target_berm_z + (DUMP_OFFSET_Z_PER_CYCLE * cycle_count as f32);
                        consecutive_replans = 0;
                        berm_exclusion_active = true;
                        current_state = RobotState::PlanToBerm; 
                    }
                } else {
                    match excavate_phase {
                        0 => {
                            path_status = format!("[CYCLE {}] EXCAVATE: CHECKING ACTUATOR...", cycle_count + 1);
                            if current_adc_reading == 0 {
                                path_status = format!("[CYCLE {}] EXCAVATE FAILED: NO ACTUATOR DATA", cycle_count + 1);
                                cmd_m5 = 0.0;
                                target_x = cfg_target_berm_x;
                                target_z = cfg_target_berm_z + (DUMP_OFFSET_Z_PER_CYCLE * cycle_count as f32);
                                consecutive_replans = 0;
                                berm_exclusion_active = true;
                                current_state = RobotState::PlanToBerm;
                            } else {
                                excavate_phase = 1;
                                target_depth_cm = Some(EXCAVATE_TARGET_DEPTH_CM);
                                excavate_belt_laps_done = 0;
                                excavate_belt_paused = false;
                                target_depo_cm = Some(current_depo_cm + EXCAVATE_BELT_INCREMENT_CM);
                                action_timer = Instant::now();
                            }
                        },

                        1 => {
                            cmd_m5 = EXCAVATE_DIG_MOTOR_EFFORT;
                            let creep = EXCAVATE_DRIVE_EFFORT * EXCAVATE_DRIVE_SPEED_SCALE;
                            cmd_m1 = -creep; cmd_m2 = -creep; cmd_m3 = creep; cmd_m4 = creep;

                            if !excavate_belt_paused {
                                path_status = format!("[CYCLE {}] EXCAVATE: DIGGING + BELT (lap {}/{})...", 
                                    cycle_count + 1, excavate_belt_laps_done, excavate_belt_total_laps);

                                let belt_settled = error_depo.abs() < DEADBAND_DEPO_CM;
                                let belt_timed_out = action_timer.elapsed().as_secs_f32() >= EXCAVATE_BELT_SETTLE_TIMEOUT_SECS;

                                if belt_settled || belt_timed_out {
                                    excavate_belt_laps_done += 1;

                                    if excavate_belt_laps_done >= excavate_belt_total_laps {
                                        cmd_m5 = 0.0;
                                        target_depo_cm = Some(current_depo_cm); 
                                        excavate_phase = 2;
                                        action_timer = Instant::now();
                                    } else {
                                        excavate_belt_paused = true;
                                        target_depo_cm = Some(current_depo_cm); 
                                        action_timer = Instant::now();
                                    }
                                }
                            } else {
                                path_status = format!("[CYCLE {}] EXCAVATE: BELT PAUSE (lap {}/{})...", 
                                    cycle_count + 1, excavate_belt_laps_done, excavate_belt_total_laps);

                                if action_timer.elapsed().as_secs_f32() >= EXCAVATE_BELT_PAUSE_SECS {
                                    excavate_belt_paused = false;
                                    target_depo_cm = Some(current_depo_cm + EXCAVATE_BELT_INCREMENT_CM);
                                    action_timer = Instant::now();
                                }
                            }
                        },

                        2 => {
                            path_status = format!("[CYCLE {}] EXCAVATE: RETRACTING...", cycle_count + 1);
                            cmd_m5 = 0.0;
                            target_depth_cm = Some(0.0);
                            target_depo_cm = Some(current_depo_cm); 
                            excavate_phase = 3;
                        },

                        3 => {
                            path_status = format!("[CYCLE {}] EXCAVATE: WAITING FOR RETRACT...", cycle_count + 1);
                            cmd_m5 = 0.0;

                            if error_ticks.abs() < DEADBAND_ADC {
                                path_status = format!("[CYCLE {}] EXCAVATION COMPLETE", cycle_count + 1);
                                target_depo_cm = Some(current_depo_cm); 
                                target_x = cfg_target_berm_x;
                                target_z = cfg_target_berm_z + (DUMP_OFFSET_Z_PER_CYCLE * cycle_count as f32);
                                consecutive_replans = 0;
                                berm_exclusion_active = true;
                                current_state = RobotState::PlanToBerm;
                            }
                        },

                        _ => {
                            cmd_m5 = 0.0;
                            target_x = cfg_target_berm_x;
                            target_z = cfg_target_berm_z + (DUMP_OFFSET_Z_PER_CYCLE * cycle_count as f32);
                            consecutive_replans = 0;
                            berm_exclusion_active = true;
                            current_state = RobotState::PlanToBerm;
                        }
                    }
                }
            },

            RobotState::PlanToBerm | RobotState::PlanToDig => {
                if consecutive_replans >= MAX_CONSECUTIVE_REPLANS {
                    path_status = String::from("TOO MANY REPLANS - STOPPING");
                    cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                    current_state = RobotState::Manual;
                    berm_exclusion_active = false;
                } else {
                    let start_grid_x = ((global_x + MAP_OFFSET) / MAP_RES).round() as isize;
                    let start_grid_z = ((global_z + MAP_OFFSET) / MAP_RES).round() as isize;
                    let goal_grid_x = ((target_x + MAP_OFFSET) / MAP_RES).round() as isize;
                    let goal_grid_z = ((target_z + MAP_OFFSET) / MAP_RES).round() as isize;
                    if let Some(path) = planner::find_manhattan_path(&arena_map, start_grid_x, start_grid_z, goal_grid_x, goal_grid_z) {
                        waypoints.clear();
                        for node in path {
                            let wp_x = (node.0 as f32 * MAP_RES) - MAP_OFFSET;
                            let wp_z = (node.1 as f32 * MAP_RES) - MAP_OFFSET;
                            waypoints.push((wp_x, wp_z));
                        }

                        if !waypoints.is_empty() { waypoints.remove(0); } 
                        if !waypoints.is_empty() {
                            turn_in_progress = false;
                            current_state = RobotState::AutoTurn;
                        } else { current_state = RobotState::Manual; }
                    } else {
                        consecutive_replans += 1;
                        path_status = format!("NO PATH FOUND (attempt {}/{})", consecutive_replans, MAX_CONSECUTIVE_REPLANS);
                    }
                }
            },

            RobotState::AutoTurn => {
                let dx = waypoints[0].0 - global_x; 
                let dz = waypoints[0].1 - global_z;
                let raw_path_yaw = f32::atan2(dx, dz);

                let lock_step = std::f32::consts::PI / 2.0; 
                let path_yaw = (raw_path_yaw / lock_step).round() * lock_step;
                
                let mut angle_diff = path_yaw - logical_yaw;
                while angle_diff > std::f32::consts::PI { angle_diff -= 2.0 * std::f32::consts::PI; }
                while angle_diff < -std::f32::consts::PI { angle_diff += 2.0 * std::f32::consts::PI; }

                if angle_diff.abs() <= 0.15 { 
                    cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                    turn_in_progress = false;
                    current_state = RobotState::AutoDrive;
                } else {
                    turn_in_progress = true;
                    let constant_turn_rpm = 80.0;
                    let turn_effort = (constant_turn_rpm * angle_diff.signum()) / base_speed; 
                    cmd_m1 = -turn_effort; cmd_m2 = -turn_effort; cmd_m3 = -turn_effort; cmd_m4 = -turn_effort;     
                }
            },
            
            RobotState::AutoDrive => {
                if waypoints.is_empty() {
                    if target_x == cfg_target_berm_x {
                        current_state = RobotState::Dump;
                        dump_phase = 0;
                        action_timer = Instant::now();
                    } else {
                        current_state = RobotState::Excavate;
                        action_timer = Instant::now();
                        excavate_phase = 0;
                        excavate_belt_laps_done = 0;
                        excavate_belt_paused = false;
                    }
                    continue;
                }

                let next_wp = waypoints[0];

                if is_path_blocked(&arena_map, global_x, global_z, next_wp.0, next_wp.1) {
                    consecutive_replans += 1;
                    path_status = format!("OBSTACLE! REPLANNING ({}/{})", consecutive_replans, MAX_CONSECUTIVE_REPLANS);
                    cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0; cmd_m5 = 0.0;
                    waypoints.clear(); 
                    
                    if target_x == cfg_target_berm_x { current_state = RobotState::PlanToBerm; } 
                    else { current_state = RobotState::PlanToDig; }
                    continue; 
                }

                let dx = next_wp.0 - global_x; let dz = next_wp.1 - global_z;
                let distance = f32::sqrt(dx*dx + dz*dz);
                let angle_to_wp = f32::atan2(dx, dz);
                let mut angle_diff = angle_to_wp - logical_yaw;
                while angle_diff > std::f32::consts::PI { angle_diff -= 2.0 * std::f32::consts::PI; }
                while angle_diff < -std::f32::consts::PI { angle_diff += 2.0 * std::f32::consts::PI; }
                let passed_waypoint = angle_diff.abs() > (std::f32::consts::PI / 2.0);
                
                if distance > 0.15 && !passed_waypoint {
                    let drive_effort = 2.0;
                    let k_yaw = 0.4;
                    let turn = (k_yaw * angle_diff).clamp(-0.35, 0.35);

                    // Forward: left motors negative, right motors positive.
                    // If this corrects the wrong way on the rover, change turn to -turn.
                    cmd_m1 = -drive_effort - turn;
                    cmd_m2 = -drive_effort - turn;
                    cmd_m3 =  drive_effort - turn;
                    cmd_m4 =  drive_effort - turn;
                } else {
                    waypoints.remove(0);
                    consecutive_replans = 0;
                    if !waypoints.is_empty() {
                        turn_in_progress = false;
                        current_state = RobotState::AutoTurn;
                    } else {
                        cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                        if target_x == cfg_target_berm_x {
                            current_state = RobotState::Dump;
                            dump_phase = 0;
                            action_timer = Instant::now();
                        } else {
                            current_state = RobotState::Excavate;
                            action_timer = Instant::now();
                            excavate_phase = 0;
                            excavate_belt_laps_done = 0;
                            excavate_belt_paused = false;
                        }
                    }
                }
            },

            RobotState::Dump => {
                if !WITH_DUMPING {
                    cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                    let elapsed = action_timer.elapsed().as_secs_f32();
                    path_status = format!("[CYCLE {}] FAKE DUMP: {:.1}s / {:.1}s...", 
                        cycle_count + 1, elapsed, FAKE_DUMP_SECS);

                    if elapsed >= FAKE_DUMP_SECS {
                        path_status = format!("[CYCLE {}] FAKE DUMP COMPLETE", cycle_count + 1);
                        berm_exclusion_active = false;
                        target_depo_cm = Some(current_depo_cm); 
                        cycle_count += 1;
                        if cycle_count < 2 {
                            target_x = cfg_target_dig_x;
                            target_z = cfg_target_dig_z;
                            consecutive_replans = 0;
                            aruco_check_phase = 0;
                            aruco_check_resume_state = 1;
                            aruco_check_got_fix = false;
                            current_state = RobotState::ArucoCheck;
                        } else {
                            path_status = String::from("MISSION COMPLETE - THANK YOU");
                            current_state = RobotState::MissionComplete;
                        }
                    }
                } else {
                    match dump_phase {
                        0 => {
                            cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                            path_status = format!("[CYCLE {}] DUMP: BELT MOVING ({:.0}cm)...", 
                                cycle_count + 1, DUMP_BELT_TRAVEL_CM);
                            target_depo_cm = Some(current_depo_cm + DUMP_BELT_TRAVEL_CM);
                            action_timer = Instant::now();
                            dump_phase = 1;
                        },

                        1 => {
                            cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                            path_status = format!("[CYCLE {}] DUMP: WAITING FOR BELT (err:{:.1}cm)...", 
                                cycle_count + 1, error_depo);

                            let belt_settled = error_depo.abs() < DEADBAND_DEPO_CM;
                            let belt_timed_out = action_timer.elapsed().as_secs_f32() >= DUMP_BELT_SETTLE_TIMEOUT_SECS;

                            if belt_settled || belt_timed_out {
                                let fwd_dx = logical_yaw.sin();
                                let fwd_dz = logical_yaw.cos();
                                let fwd_clearance = distance_to_boundary(global_x, global_z, fwd_dx, fwd_dz, cfg_bounds_x_min, cfg_bounds_x_max, cfg_bounds_z_min, cfg_bounds_z_max);
                                let bwd_clearance = distance_to_boundary(global_x, global_z, -fwd_dx, -fwd_dz, cfg_bounds_x_min, cfg_bounds_x_max, cfg_bounds_z_min, cfg_bounds_z_max);
                                dump_backout_forward = fwd_clearance >= bwd_clearance;

                                let chosen_dir = if dump_backout_forward { "FWD" } else { "REV" };
                                path_status = format!("[CYCLE {}] DUMP: BELT COMPLETE — BACKING OUT ({}, fwd:{:.1}m bwd:{:.1}m)...", 
                                    cycle_count + 1, chosen_dir, fwd_clearance, bwd_clearance);
                                dump_backout_start_x = global_x;
                                dump_backout_start_z = global_z;
                                target_depo_cm = Some(current_depo_cm);
                                dump_phase = 2;
                            }
                        },

                        2 => {
                            let dx_backout = global_x - dump_backout_start_x;
                            let dz_backout = global_z - dump_backout_start_z;
                            let dist_backed = f32::sqrt(dx_backout * dx_backout + dz_backout * dz_backout);

                            let dir_label = if dump_backout_forward { "FWD" } else { "REV" };
                            path_status = format!("[CYCLE {}] DUMP: BACKING OUT {} ({:.2}/{:.2}m)...", 
                                cycle_count + 1, dir_label, dist_backed, DUMP_BACKOUT_DISTANCE_M);

                            if dist_backed >= DUMP_BACKOUT_DISTANCE_M {
                                cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                                dump_phase = 3;
                            } else {
                                let backout_effort = DUMP_BACKOUT_EFFORT * DUMP_BACKOUT_SPEED_SCALE;
                                if dump_backout_forward {
                                    cmd_m1 = -backout_effort; cmd_m2 = -backout_effort; 
                                    cmd_m3 = backout_effort; cmd_m4 = backout_effort;
                                } else {
                                    cmd_m1 = backout_effort; cmd_m2 = backout_effort; 
                                    cmd_m3 = -backout_effort; cmd_m4 = -backout_effort;
                                }
                            }
                        },

                        3 => {
                            cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                            berm_exclusion_active = false;
                            target_depo_cm = Some(current_depo_cm); 

                            cycle_count += 1;
                            if cycle_count < 2 {
                                path_status = format!("[CYCLE {}] DUMP COMPLETE — HEADING TO DIG", cycle_count + 1);
                                target_x = cfg_target_dig_x;
                                target_z = cfg_target_dig_z;
                                consecutive_replans = 0;
                                aruco_check_phase = 0;
                                aruco_check_resume_state = 1;
                                aruco_check_got_fix = false;
                                current_state = RobotState::ArucoCheck;
                            } else {
                                path_status = String::from("MISSION COMPLETE - THANK YOU");
                                current_state = RobotState::MissionComplete;
                            }
                        },

                        _ => {
                            cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                            berm_exclusion_active = false;
                            target_depo_cm = Some(current_depo_cm);
                            cycle_count += 1;
                            if cycle_count < 2 {
                                target_x = cfg_target_dig_x;
                                target_z = cfg_target_dig_z;
                                consecutive_replans = 0;
                                aruco_check_phase = 0;
                                aruco_check_resume_state = 1;
                                aruco_check_got_fix = false;
                                current_state = RobotState::ArucoCheck;
                            } else {
                                path_status = String::from("MISSION COMPLETE - THANK YOU");
                                current_state = RobotState::MissionComplete;
                            }
                        }
                    }
                }
            },

            RobotState::ArucoCheck => {
                match aruco_check_phase {
                    0 => {
                        cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                        aruco_check_pre_yaw = logical_yaw;
                        let yaw_away_from_marker = f32::atan2(global_x, global_z);
                        let lock_step = std::f32::consts::PI / 2.0;
                        let target_yaw = (yaw_away_from_marker / lock_step).round() * lock_step;
                        turn_target_pos = target_yaw;
                        while turn_target_pos > std::f32::consts::PI { turn_target_pos -= 2.0 * std::f32::consts::PI; }
                        while turn_target_pos < -std::f32::consts::PI { turn_target_pos += 2.0 * std::f32::consts::PI; }
                        
                        aruco_check_got_fix = false;
                        path_status = format!("ARUCO CHECK: TURNING TO FACE MARKER (yaw {:.2} -> {:.2})", logical_yaw, turn_target_pos);
                        aruco_check_phase = 1;
                    },

                    1 => {
                        let mut angle_diff = turn_target_pos - logical_yaw;
                        while angle_diff > std::f32::consts::PI { angle_diff -= 2.0 * std::f32::consts::PI; }
                        while angle_diff < -std::f32::consts::PI { angle_diff += 2.0 * std::f32::consts::PI; }

                        if angle_diff.abs() <= 0.15 {
                            cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                            aruco_check_timer = Instant::now();
                            path_status = String::from("ARUCO CHECK: WAITING FOR DETECTION...");
                            aruco_check_phase = 2;
                        } else {
                            let constant_turn_rpm = 160.0;
                            let turn_effort = (constant_turn_rpm * angle_diff.signum()) / base_speed;
                            cmd_m1 = -turn_effort; cmd_m2 = -turn_effort; 
                            cmd_m3 = -turn_effort; cmd_m4 = -turn_effort;
                        }
                    },

                    2 => {
                        cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                        let elapsed = aruco_check_timer.elapsed().as_secs_f32();

                        if aruco_check_got_fix || elapsed >= ARUCO_CHECK_TIMEOUT_SECS {
                            match aruco_check_resume_state {
                                0 => { current_state = RobotState::PlanToBerm; },
                                1 => { current_state = RobotState::PlanToDig; },
                                2 => {
                                    current_state = RobotState::Excavate;
                                    action_timer = Instant::now();
                                    excavate_phase = 0;
                                    excavate_belt_laps_done = 0;
                                    excavate_belt_paused = false;
                                },
                                _ => { current_state = RobotState::Manual; },
                            }
                        } else {
                            path_status = format!("ARUCO CHECK: WAITING...");
                        }
                    },

                    _ => {
                        cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                        current_state = RobotState::Manual;
                    }
                }
            },

            RobotState::TestExcavate => {
                cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;

                match excavate_phase {
                    0 => {
                        path_status = String::from("[TEST] EXCAVATE: CHECKING ACTUATOR...");
                        if current_adc_reading == 0 {
                            path_status = String::from("[TEST] EXCAVATE FAILED: NO ACTUATOR DATA");
                            cmd_m5 = 0.0;
                            current_state = RobotState::Manual;
                        } else {
                            excavate_phase = 1;
                            target_depth_cm = Some(EXCAVATE_TARGET_DEPTH_CM);
                            excavate_belt_laps_done = 0;
                            excavate_belt_paused = false;
                            target_depo_cm = Some(current_depo_cm + EXCAVATE_BELT_INCREMENT_CM);
                            action_timer = Instant::now();
                        }
                    },
                    1 => {
                        cmd_m5 = EXCAVATE_DIG_MOTOR_EFFORT;
                        let creep = EXCAVATE_DRIVE_EFFORT * EXCAVATE_DRIVE_SPEED_SCALE;
                        cmd_m1 = -creep; cmd_m2 = -creep; cmd_m3 = creep; cmd_m4 = creep;

                        if !excavate_belt_paused {
                            path_status = format!("[TEST] EXCAVATE: DIGGING + BELT (lap {}/{})...", 
                                excavate_belt_laps_done, excavate_belt_total_laps);

                            let belt_settled = error_depo.abs() < DEADBAND_DEPO_CM;
                            let belt_timed_out = action_timer.elapsed().as_secs_f32() >= EXCAVATE_BELT_SETTLE_TIMEOUT_SECS;

                            if belt_settled || belt_timed_out {
                                excavate_belt_laps_done += 1;

                                if excavate_belt_laps_done >= excavate_belt_total_laps {
                                    cmd_m5 = 0.0;
                                    target_depo_cm = Some(current_depo_cm);
                                    excavate_phase = 2;
                                    action_timer = Instant::now();
                                } else {
                                    excavate_belt_paused = true;
                                    target_depo_cm = Some(current_depo_cm);
                                    action_timer = Instant::now();
                                }
                            }
                        } else {
                            path_status = format!("[TEST] EXCAVATE: BELT PAUSE (lap {}/{})...", 
                                excavate_belt_laps_done, excavate_belt_total_laps);

                            if action_timer.elapsed().as_secs_f32() >= EXCAVATE_BELT_PAUSE_SECS {
                                excavate_belt_paused = false;
                                target_depo_cm = Some(current_depo_cm + EXCAVATE_BELT_INCREMENT_CM);
                                action_timer = Instant::now();
                            }
                        }
                    },
                    2 => {
                        path_status = String::from("[TEST] EXCAVATE: RETRACTING...");
                        cmd_m5 = 0.0;
                        target_depth_cm = Some(0.0);
                        target_depo_cm = Some(current_depo_cm); 
                        excavate_phase = 3;
                    },
                    3 => {
                        path_status = String::from("[TEST] EXCAVATE: WAITING FOR RETRACT...");
                        cmd_m5 = 0.0;

                        if error_ticks.abs() < DEADBAND_ADC {
                            path_status = String::from("[TEST] EXCAVATION COMPLETE");
                            target_depo_cm = Some(current_depo_cm);
                            current_state = RobotState::Manual;
                        }
                    },
                    _ => {
                        cmd_m5 = 0.0;
                        current_state = RobotState::Manual;
                    }
                }
            },

            RobotState::TestDump => {
                match dump_phase {
                    0 => {
                        let dx_in = global_x - dump_backout_start_x;
                        let dz_in = global_z - dump_backout_start_z;
                        let dist_in = f32::sqrt(dx_in * dx_in + dz_in * dz_in);

                        path_status = format!("[TEST] DUMP: GOING IN ({:.2}/{:.2}m)...", dist_in, DUMP_IN_DISTANCE_M);

                        if dist_in >= DUMP_IN_DISTANCE_M {
                            cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                            dump_phase = 1;
                        } else {
                            let in_effort = DUMP_IN_EFFORT * DUMP_IN_SPEED_SCALE;
                            cmd_m1 = in_effort; cmd_m2 = in_effort; 
                            cmd_m3 = -in_effort; cmd_m4 = -in_effort;
                        }
                    },

                    1 => {
                        cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                        path_status = format!("[TEST] DUMP: BELT MOVING ({:.0}cm)...", DUMP_BELT_TRAVEL_CM);
                        target_depo_cm = Some(current_depo_cm + DUMP_BELT_TRAVEL_CM);
                        action_timer = Instant::now();
                        dump_phase = 2;
                    },

                    2 => {
                        cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                        path_status = format!("[TEST] DUMP: WAITING FOR BELT (err:{:.1}cm)...", error_depo);

                        let belt_settled = error_depo.abs() < DEADBAND_DEPO_CM;
                        let belt_timed_out = action_timer.elapsed().as_secs_f32() >= DUMP_BELT_SETTLE_TIMEOUT_SECS;

                        if belt_settled || belt_timed_out {
                            let fwd_dx = logical_yaw.sin();
                            let fwd_dz = logical_yaw.cos();
                            let fwd_clearance = distance_to_boundary(global_x, global_z, fwd_dx, fwd_dz, cfg_bounds_x_min, cfg_bounds_x_max, cfg_bounds_z_min, cfg_bounds_z_max);
                            let bwd_clearance = distance_to_boundary(global_x, global_z, -fwd_dx, -fwd_dz, cfg_bounds_x_min, cfg_bounds_x_max, cfg_bounds_z_min, cfg_bounds_z_max);
                            dump_backout_forward = fwd_clearance >= bwd_clearance;
                            let chosen_dir = if dump_backout_forward { "FWD" } else { "REV" };
                            path_status = format!("[TEST] DUMP: BELT COMPLETE — BACKING OUT ({}, fwd:{:.1}m bwd:{:.1}m)...", 
                                chosen_dir, fwd_clearance, bwd_clearance);
                            dump_backout_start_x = global_x;
                            dump_backout_start_z = global_z;
                            target_depo_cm = Some(current_depo_cm); 
                            dump_phase = 3;
                        }
                    },

                    3 => {
                        let dx_backout = global_x - dump_backout_start_x;
                        let dz_backout = global_z - dump_backout_start_z;
                        let dist_backed = f32::sqrt(dx_backout * dx_backout + dz_backout * dz_backout);

                        let dir_label = if dump_backout_forward { "FWD" } else { "REV" };
                        path_status = format!("[TEST] DUMP: BACKING OUT {} ({:.2}/{:.2}m)...", 
                            dir_label, dist_backed, DUMP_BACKOUT_DISTANCE_M);

                        if dist_backed >= DUMP_BACKOUT_DISTANCE_M {
                            cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                            dump_phase = 4;
                        } else {
                            let backout_effort = DUMP_BACKOUT_EFFORT * DUMP_BACKOUT_SPEED_SCALE;
                            if dump_backout_forward {
                                cmd_m1 = -backout_effort; cmd_m2 = -backout_effort; 
                                cmd_m3 = backout_effort; cmd_m4 = backout_effort;
                            } else {
                                cmd_m1 = backout_effort; cmd_m2 = backout_effort; 
                                cmd_m3 = -backout_effort; cmd_m4 = -backout_effort;
                            }
                        }
                    },

                    4 => {
                        cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                        target_depo_cm = Some(current_depo_cm);
                        path_status = String::from("[TEST] DUMP COMPLETE");
                        current_state = RobotState::Manual;
                    },

                    _ => {
                        cmd_m1 = 0.0; cmd_m2 = 0.0; cmd_m3 = 0.0; cmd_m4 = 0.0;
                        target_depo_cm = Some(current_depo_cm);
                        current_state = RobotState::Manual;
                    }
                }
            }
        }

        let send_rpm_1 = base_speed * cmd_m1; let send_rpm_2 = base_speed * cmd_m2;
        let send_rpm_3 = base_speed * cmd_m3; let send_rpm_4 = base_speed * cmd_m4;
        let send_rpm_5 = base_speed * cmd_m5;

        let sim_msg = SimCommand { 
            m1: cmd_m1, m2: cmd_m2, m3: cmd_m3, m4: cmd_m4,
            x: global_x, z: global_z, yaw: logical_yaw 
        };
        if let Ok(json_msg) = serde_json::to_string(&sim_msg) {
            let _ = publisher.send(&json_msg, 0);
        }

        if ENABLE_MOTOR_1 { send_velocity_command(&tx_socket, 1, send_rpm_1, INVERT_MOTOR_1); }
        if ENABLE_MOTOR_2 { send_velocity_command(&tx_socket, 2, send_rpm_2, INVERT_MOTOR_2); }
        if ENABLE_MOTOR_3 { send_velocity_command(&tx_socket, 3, send_rpm_3, INVERT_MOTOR_3); }
        if ENABLE_MOTOR_4 { send_velocity_command(&tx_socket, 4, send_rpm_4, INVERT_MOTOR_4); }
        if ENABLE_MOTOR_5 { send_velocity_command(&tx_socket, 5, send_rpm_5, INVERT_MOTOR_5); } 

        send_sabertooth_command(&tx_socket, act_speed, depo_speed);

        let direction_debug = if cmd_m1 == 0.0 && cmd_m2 == 0.0 && cmd_m3 == 0.0 && cmd_m4 == 0.0 { "STOPPED" } 
        else if cmd_m1 < 0.0 && cmd_m2 < 0.0 && cmd_m3 > 0.0 && cmd_m4 > 0.0 { "FORWARD" } 
        else if cmd_m1 > 0.0 && cmd_m2 > 0.0 && cmd_m3 < 0.0 && cmd_m4 < 0.0 { "BACKWARD" } 
        else if cmd_m1 > 0.0 && cmd_m2 > 0.0 && cmd_m3 > 0.0 && cmd_m4 > 0.0 { "TURNING LEFT" } 
        else if cmd_m1 < 0.0 && cmd_m2 < 0.0 && cmd_m3 < 0.0 && cmd_m4 < 0.0 { "TURNING RIGHT" } 
        else { "MIXED" };

        if print_timer.elapsed().as_millis() > 100 {
            execute!(io::stdout(), MoveTo(0, 0)).unwrap();
            if show_map {
                let center_x = ((global_x + MAP_OFFSET) / MAP_RES).round() as isize;
                let center_z = ((global_z + MAP_OFFSET) / MAP_RES).round() as isize;
                let mut map_str = String::new();
                map_str.push_str("\r\n=== LIVE OBSTACLE MAP & PATH ===\r\n");

                for z in (center_z - 20..=center_z + 20).rev() { 
                    for x in center_x - 20..=center_x + 20 {
                        let is_waypoint = waypoints.iter().any(|&(wx, wz)| {
                            let grid_wx = ((wx + MAP_OFFSET) / MAP_RES).round() as isize;
                            let grid_wz = ((wz + MAP_OFFSET) / MAP_RES).round() as isize;
                            x == grid_wx && z == grid_wz
                        });

                        if x == center_x && z == center_z { map_str.push_str("R "); } 
                        else if is_waypoint { map_str.push_str("* "); } 
                        else if x >= 0 && x < MAP_SIZE as isize && z >= 0 && z < MAP_SIZE as isize {
                            let conf = arena_map[x as usize][z as usize];
                            if conf >= CONFIDENCE_THRESHOLD { map_str.push_str("X "); } 
                            else if conf > 0 { map_str.push_str("· "); }
                            else { map_str.push_str(". "); }
                        } else { map_str.push_str("  "); }
                    }
                    map_str.push_str("\r\n");
                }
                print!("{}", map_str);
            } else {
                let actual_cm = (current_adc_reading as f32 - CAL_B) / CAL_M;
                let tgt_act = target_depth_cm.unwrap_or(actual_cm);
                let tgt_dep = target_depo_cm.unwrap_or(current_depo_cm);
                
                print!("MODE: {:15} | POS X:{:5.2} Z:{:5.2} | L-YAW:{:5.2}\r\nSYS: {:15} | CYCLE: {}/2 | PATH: {:35}\r\nDIR: {:15} | CMD: M1:{:4.0} M2:{:4.0} M3:{:4.0} M4:{:4.0} M5:{:4.0} \r\nENC: V_FWD:{:5.2} m/s | ACT: {:.1}cm -> {:.1}cm | DEPO: {:.1}cm -> {:.1}cm\r\nIMU: YAW:{:7.2}° PITCH:{:7.2}° ROLL:{:7.2}°\r\n", 
                    loc_mode, global_x, global_z, logical_yaw, format!("{:?}", current_state), cycle_count, path_status,
                    direction_debug, send_rpm_1, send_rpm_2, send_rpm_3, send_rpm_4, send_rpm_5, encoder_v_forward, actual_cm, tgt_act, current_depo_cm, tgt_dep,
                    imu_yaw_deg, imu_pitch_deg, imu_roll_deg);
                println!("CONTROLS: 'L'=Loc | 'G'=AUTO | 'M'=Stop | 'o'/'k'=Actuator | 'z'/'y'=Deposition | '1'/'2'/'3'=Test Turns | '7'=Test Excavate | '9'=Test Dump");
            }
            io::stdout().flush().unwrap();
            print_timer = Instant::now();
        }
        thread::sleep(Duration::from_millis(20));
    }
    disable_raw_mode().expect("Failed to disable raw mode");

    println!("\r\nSafely shutting down all motors...");
    send_velocity_command(&tx_socket, 1, 0.0, false);
    send_velocity_command(&tx_socket, 2, 0.0, false);
    send_velocity_command(&tx_socket, 3, 0.0, false);
    send_velocity_command(&tx_socket, 4, 0.0, false);
    send_velocity_command(&tx_socket, 5, 0.0, false);
    send_sabertooth_command(&tx_socket, 0, 0);
    thread::sleep(Duration::from_millis(150)); 
}