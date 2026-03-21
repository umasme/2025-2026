# UM ASME
# Electrical and software subteams
# By Juan and Ahmed

import pyrealsense2 as rs
import numpy as np
import zmq
import json

# Setup ZeroMQ Publisher
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")

latest_accel = np.zeros(3)
latest_gyro = np.zeros(3)

def imu_callback(frame):
    global latest_accel, latest_gyro
    if frame.is_motion_frame():
        motion = frame.as_motion_frame()
        motion_data = motion.get_motion_data()
        
        if frame.get_profile().stream_type() == rs.stream.accel:
            latest_accel = np.array([motion_data.x, motion_data.y, motion_data.z])
        elif frame.get_profile().stream_type() == rs.stream.gyro:
            latest_gyro = np.array([motion_data.x, motion_data.y, motion_data.z])

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

print("Starting Video Pipeline...")
profile = pipeline.start(config)

print("Tapping directly into IMU Hardware...")
device = profile.get_device()
imu_sensor = None

for sensor in device.query_sensors():
    if sensor.is_motion_sensor():
        imu_sensor = sensor
        break

if imu_sensor:
    imu_profiles = imu_sensor.get_stream_profiles()
    accel_profile = next(p for p in imu_profiles if p.stream_type() == rs.stream.accel)
    gyro_profile = next(p for p in imu_profiles if p.stream_type() == rs.stream.gyro)

    imu_sensor.open([accel_profile, gyro_profile])
    imu_sensor.start(imu_callback)
    print("IMU Sensor running at max frequency in the background!")
else:
    print("Error: Could not find IMU hardware on this device.")

pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2)

try:
    while True:
        frames = pipeline.wait_for_frames()
        
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame: 
            continue

        depth_frame = decimate.process(depth_frame)
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)

        v = points.get_vertices()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3) 

        close_points = verts[verts[:, 2] < 1.0]
        if len(close_points) > 500:
            print(f"Warning: {len(close_points)} obstacle points detected nearby!")

        # Broadcast the IMU data to the Rust middleware
        payload = {
            "accel": latest_accel.tolist(),
            "gyro": latest_gyro.tolist()
        }
        socket.send_string(json.dumps(payload))

        print(f"Broadcasting -> Accel [m/s^2]: X: {latest_accel[0]:5.2f}, Y: {latest_accel[1]:5.2f}, Z: {latest_accel[2]:5.2f} | "
              f"Gyro [rad/s]: X: {latest_gyro[0]:5.2f}, Y: {latest_gyro[1]:5.2f}, Z: {latest_gyro[2]:5.2f}")

except KeyboardInterrupt:
    print("\nProgram interrupted.")
finally:
    if imu_sensor:
        try:
            imu_sensor.stop()
            imu_sensor.close()
        except RuntimeError:
            pass
    socket.close()
    context.term()
    pipeline.stop()
