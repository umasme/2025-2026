use socketcan::{CanSocket, Socket, ExtendedId, CanFrame, Frame, Id, EmbeddedFrame};
use std::io::{self, Write};
use std::thread;
use std::time::Duration;
use std::f32::consts::PI;

// ==========================================
// ROVER CONFIGURATION
// ==========================================
// IMPORTANT 0.837438 FOR 1 REVOLUTION
const INVERT_MOTOR_1: bool = false;
const INVERT_MOTOR_2: bool = false;
const INVERT_MOTOR_3: bool = false;
const INVERT_MOTOR_4: bool = false;

const GEAR_RATIO: f32 = 64.0; // gear ratio
const WHEEL_DIAMETER_INCHES: f32 = 10.5; // change it 
// 1 inch = 0.0254 meters
const WHEEL_CIRCUMFERENCE_METERS: f32 = (WHEEL_DIAMETER_INCHES * 0.0254) * PI;
fn build_velocity_can_id(device_id: u32) -> ExtendedId {
    let device_type = 2 << 24;
    let manufacturer = 5 << 16;
    let api_class = 1 << 10; 
    let api_index = 2 << 6;  
    let can_id = device_type | manufacturer | api_class | api_index | device_id;
    ExtendedId::new(can_id).expect("Failed to create ExtendedId for Velocity Command")
}

/// Sends a velocity (RPM) command to a specific motor
fn send_velocity_command(socket: &CanSocket, device_id: u32, rpm: f32, invert: bool) {
    let mut final_rpm = rpm;
    if invert { final_rpm = -final_rpm; }
    let id = build_velocity_can_id(device_id);
    let rpm_bytes = final_rpm.to_le_bytes();
    let payload: [u8; 8] = [
        rpm_bytes[0], rpm_bytes[1], rpm_bytes[2], rpm_bytes[3],
        0, 0, 0, 1 
    ];
    let frame = CanFrame::new(id, &payload).expect("Failed to construct CAN frame");
    let _ = socket.write_frame(&frame);
}

/// Executes a P-Controller move based on reading the position of Motor 1
fn execute_position_move(rx_socket: &CanSocket, tx_socket: &CanSocket, motor_revolutions_to_move: f32) {
    let mut current_position_m1: f32 = 0.0;
    let mut got_initial_position = false;
    
    println!("Waiting to read current position from Motor 1 on CAN bus...");

    // 1. Wait until we get an initial position reading from Motor 1
    while !got_initial_position {
        while let Ok(frame) = rx_socket.read_frame() {
            if let Id::Extended(ext_id) = frame.id() {
                let can_id = ext_id.as_raw();
                let device_id = can_id & 0x3F;
                let api_index = (can_id >> 6) & 0xF;
                let api_class = (can_id >> 10) & 0x3F;
                let manufacturer = (can_id >> 16) & 0xFF;

                // Match SPARK MAX Periodic Status 2 (Class 6, Index 2) for Motor 1
                if manufacturer == 5 && api_class == 6 && api_index == 2 && device_id == 1 {
                    let payload = frame.data();
                    if payload.len() >= 4 {
                        let mut pos_bytes = [0u8; 4];
                        pos_bytes.copy_from_slice(&payload[0..4]);
                        current_position_m1 = f32::from_le_bytes(pos_bytes);
                        got_initial_position = true;
                    }
                }
            }
        }
        if !got_initial_position {
            thread::sleep(Duration::from_millis(5)); 
        }
    }

    // 2. Set targets and control parameters
    let target_position = current_position_m1 + motor_revolutions_to_move;
    
    // --- P-CONTROLLER TUNING ---
    let kp = 50.0;        // Proportional gain
    let max_rpm = 800.0;  // Max speed limit
    let min_rpm = 40.0;   // Minimum speed to overcome physical friction
    let tolerance = 0.15; // Tolerance in motor revolutions (0.15 revs = ~1.8 degrees of wheel turn)

    println!("Position Locked! Initial: {:.2} revs | Target: {:.2} revs", current_position_m1, target_position);

    // 3. Control Loop
    loop {
        // Clear the receive buffer and get the absolute latest position
        while let Ok(frame) = rx_socket.read_frame() {
            if let Id::Extended(ext_id) = frame.id() {
                let can_id = ext_id.as_raw();
                let device_id = can_id & 0x3F;
                let api_index = (can_id >> 6) & 0xF;
                let api_class = (can_id >> 10) & 0x3F;
                let manufacturer = (can_id >> 16) & 0xFF;

                if manufacturer == 5 && api_class == 6 && api_index == 2 && device_id == 1 {
                    let payload = frame.data();
                    if payload.len() >= 4 {
                        let mut pos_bytes = [0u8; 4];
                        pos_bytes.copy_from_slice(&payload[0..4]);
                        current_position_m1 = f32::from_le_bytes(pos_bytes);
                    }
                }
            }
        }

        // Calculate Error
        let error = target_position - current_position_m1;

        // Check if target is reached
        if error.abs() <= tolerance {
            send_velocity_command(tx_socket, 1, 0.0, INVERT_MOTOR_1);
            send_velocity_command(tx_socket, 2, 0.0, INVERT_MOTOR_2);
            send_velocity_command(tx_socket, 3, 0.0, INVERT_MOTOR_3);
            send_velocity_command(tx_socket, 4, 0.0, INVERT_MOTOR_4);
            println!("\n>>> Target Reached! Final Pos: {:.2} | Error: {:.3} revs <<<\n", current_position_m1, error);
            break;
        }

        // Calculate Command RPM with Deadband Compensation
        let mut cmd_rpm = kp * error;
        if cmd_rpm.abs() < min_rpm {
            cmd_rpm = min_rpm * error.signum(); 
        }
        cmd_rpm = cmd_rpm.clamp(-max_rpm, max_rpm);

        // Send to motors
        send_velocity_command(tx_socket, 1, cmd_rpm, INVERT_MOTOR_1);
        send_velocity_command(tx_socket, 2, cmd_rpm, INVERT_MOTOR_2);
        send_velocity_command(tx_socket, 3, cmd_rpm, INVERT_MOTOR_3);
        send_velocity_command(tx_socket, 4, cmd_rpm, INVERT_MOTOR_4);

        // Display Telemetry
        print!("\rTarget: {:.2} | Current: {:.2} | Error: {:.2} | Cmd RPM: {:6.0}   ", 
                 target_position, current_position_m1, error, cmd_rpm);
        io::stdout().flush().unwrap();

        // Loop runs at ~50Hz
        thread::sleep(Duration::from_millis(20));
    }
}

fn main() {
    let rx_socket = CanSocket::open("can0").expect("CRITICAL: Failed to open CAN rx_socket");
    rx_socket.set_nonblocking(true).expect("Failed to set CAN socket to non-blocking");
    let tx_socket = CanSocket::open("can0").expect("CRITICAL: Failed to open CAN tx_socket");

    // ====================================================================
    // HEARTBEAT THREAD (Keeps SPARK MAX/Flex controllers enabled)
    // ====================================================================
    let heartbeat_rx_socket = CanSocket::open("can0").expect("Failed to open heartbeat socket");
    heartbeat_rx_socket.set_nonblocking(true).expect("Failed to set heartbeat socket to non-blocking");

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
    // ====================================================================

    // Stop motors initially
    send_velocity_command(&tx_socket, 1, 0.0, INVERT_MOTOR_1);
    send_velocity_command(&tx_socket, 2, 0.0, INVERT_MOTOR_2);
    send_velocity_command(&tx_socket, 3, 0.0, INVERT_MOTOR_3);
    send_velocity_command(&tx_socket, 4, 0.0, INVERT_MOTOR_4);

    loop {
        println!("========================================");
        println!("       UMRS ROVER COMMAND DASHBOARD     ");
        println!("========================================");
        println!("[1] Move by Motor Revolutions");
        println!("[2] Move by Distance (Meters)");
        println!("[q] Quit");
        print!("Select mode: ");
        io::stdout().flush().unwrap();

        let mut mode_input = String::new();
        io::stdin().read_line(&mut mode_input).unwrap();
        let mode_input = mode_input.trim();

        if mode_input.eq_ignore_ascii_case("q") {
            println!("Shutting down rover safely...");
            break;
        }

        if mode_input == "1" {
            print!("Enter wheel revolutions to move (e.g. 10.5 or -5): ");
            io::stdout().flush().unwrap();
            
            let mut val_input = String::new();
            io::stdin().read_line(&mut val_input).unwrap();
            
            if let Ok(revs) = val_input.trim().parse::<f32>() {
                println!("Commanding {} wheel revolutions...", revs);
                let motor_revs =  revs* GEAR_RATIO;

                execute_position_move(&rx_socket, &tx_socket, motor_revs);
            } else {
                println!("Invalid input. Please enter a valid number.\n");
            }

        } else if mode_input == "2" {
            print!("Enter distance to move in METERS (e.g. 1.5 or -0.5): ");
            io::stdout().flush().unwrap();
            
            let mut val_input = String::new();
            io::stdin().read_line(&mut val_input).unwrap();
            
            if let Ok(meters) = val_input.trim().parse::<f32>() {
                // MATH:
                // 1 Wheel Rev = WHEEL_CIRCUMFERENCE_METERS
                // Total Wheel Revs needed = meters / WHEEL_CIRCUMFERENCE
                // Motor Revs needed = Wheel Revs * GEAR_RATIO
                
                let wheel_revs = meters / WHEEL_CIRCUMFERENCE_METERS;
                let motor_revs = wheel_revs * GEAR_RATIO;
                
                println!("Calculated Math: {:.2} meters = {:.2} wheel revs = {:.2} motor revs", meters, wheel_revs, motor_revs);
                execute_position_move(&rx_socket, &tx_socket, motor_revs);
            } else {
                println!("Invalid input. Please enter a valid number.\n");
            }
        } else {
            println!("Invalid selection. Press 1, 2, or q.\n");
        }
    }
}