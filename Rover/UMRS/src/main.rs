use serde::{Deserialize, Serialize};
use std::time::Instant;

mod kalman;

#[derive(Serialize, Deserialize, Debug)]
struct ImuData {
    accel: Vec<f32>,
    gyro: Vec<f32>,
}

fn main() {
    let context = zmq::Context::new();
    let subscriber = context.socket(zmq::SUB).unwrap();
    subscriber.connect("tcp://localhost:5555").expect("Failed to connect");
    subscriber.set_subscribe(b"").expect("Failed to subscribe");

    // Initialize separate filters for Pitch and Yaw
    let mut pitch_filter = kalman::KalmanFilter::new();
    let mut yaw_filter = kalman::KalmanFilter::new();
    
    let mut last_time = Instant::now();

    println!("Middleware Active: Tracking Pitch and Yaw...");

    loop {
        let msg = subscriber.recv_string(0).unwrap().unwrap();
        let data: ImuData = serde_json::from_str(&msg).unwrap();

        let dt = last_time.elapsed().as_secs_f32();
        last_time = Instant::now();

        // --- PITCH ---
        let accel_pitch = f32::atan2(data.accel[2], -data.accel[1]);
        pitch_filter.predict(data.gyro[0], dt); 
        pitch_filter.update(accel_pitch);

        // --- YAW ---
        // Yaw is rotation around Y-axis. No accel correction yet (stationary).
        yaw_filter.predict(data.gyro[1], dt); 

        println!(
            "PITCH: {:5.2}° | YAW (Heading): {:5.2}°", 
            pitch_filter.get_angle().to_degrees(), 
            yaw_filter.get_angle().to_degrees()
        );
    }
}
