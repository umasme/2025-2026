import pyrealsense2 as rs
import numpy as np
import zmq
import json
import cv2
import math
import time

headless = True
USE_REAR_CAMERA = True

FRONT_CAMERA_SERIAL = '327122073351' 
REAR_CAMERA_SERIAL  = '247122073398'
MARKER_SIZE_METERS = 0.20  
MIN_DISTANCE = 0.4   
MAX_DISTANCE = 3.0
NUM_COLUMN_BANDS = 16 
SCAN_ROW_STEP = 4     
OBSTACLE_HEIGHT_THRESHOLD = 0.1 
DEPTH_JUMP_THRESHOLD = 0.10 
MIN_OBSTACLE_PIXELS = 8
MIN_FLOOR_PIXELS = 5
HISTORY_FRAMES = 5
CONFIRM_FRAMES = 3
VIEW_WIDTH = 2.4

MARKER_GLOBAL_X = 0.0  
MARKER_GLOBAL_Z = 0.0
if USE_REAR_CAMERA:
    print("=============================================")
    print("    ARENA CONFIGURATION — ArUco Position     ")
    print("=============================================")
    print("  Option 1 (Default): ArUco at (0, 0)")
    print("  Option 2 (Secondary): ArUco at (0, 6)")
    print("=============================================")
    arena_choice = input("Select (1/2): ").strip()
    if arena_choice == "2":
        MARKER_GLOBAL_X = 0.0
        MARKER_GLOBAL_Z = 6.0
        print(f"=> ArUco position set to ({MARKER_GLOBAL_X}, {MARKER_GLOBAL_Z})")
    else:
        MARKER_GLOBAL_X = 0.0
        MARKER_GLOBAL_Z = 0.0
        print(f"=> ArUco position set to ({MARKER_GLOBAL_X}, {MARKER_GLOBAL_Z})")
else:
    print("Rear camera (ArUco) disabled — skipping arena config.")

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

print("Initializing RealSense Cameras...")
pipelines = {}
depth_intrinsics = None 

try:
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        serial = dev.get_info(rs.camera_info.serial_number)
        print(f"Found Device: {serial}")
        
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        
        if serial == FRONT_CAMERA_SERIAL:
            print("Configuring Front Camera (Depth + RGB)")
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
        elif serial == REAR_CAMERA_SERIAL and USE_REAR_CAMERA:
            print("Configuring Rear Camera (RGB Only for ArUco)")
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        else:
            print(f"Skipping device {serial}")
            continue
            
        profile = pipeline.start(config)
        pipelines[serial] = pipeline
        if serial == FRONT_CAMERA_SERIAL:
            depth_stream = profile.get_stream(rs.stream.depth)
            depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
            print(f"Depth Intrinsics: fx={depth_intrinsics.fx:.1f} fy={depth_intrinsics.fy:.1f} "
                  f"ppx={depth_intrinsics.ppx:.1f} ppy={depth_intrinsics.ppy:.1f}")
        
except Exception as e:
    print(f"CRITICAL ERROR initializing cameras: {e}")

camera_matrix = np.array([[615.0, 0, 320.0],
                          [0, 615.0, 240.0],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4,1))

print("\n--- Perception System Active (Per-Column Floor Scan) ---")
obstacle_history = []


def deproject_pixel(row, col, depth_m, intrin):
    """
    Convert a depth image pixel (row, col) + depth to a 3D point (x, y, z)
    in the camera coordinate frame.
    
    Camera frame convention (RealSense):
      X = right
      Y = down  
      Z = forward (into the scene)
    """
    x = (col - intrin.ppx) / intrin.fx * depth_m
    y = (row - intrin.ppy) / intrin.fy * depth_m
    z = depth_m
    return x, y, z


def scan_columns_for_obstacles(depth_image, intrin):
    """
    Per-column floor scanning obstacle detection.
    
    For each of the NUM_COLUMN_BANDS vertical bands:
      1. Scan from the BOTTOM of the image upward (row 479 → 0).
      2. The bottom rows should be the closest floor. Establish a floor
         baseline from the first few valid depth pixels.
      3. Continue scanning upward. For each pixel, deproject to 3D and
         check two conditions:
         a) Did the depth suddenly DECREASE? (object closer than floor)
         b) Is the 3D Y value significantly LOWER than predicted? 
            (In camera frame Y=down, so "higher in world" = smaller Y.
             An obstacle sticking up has a LOWER Y than the floor at 
             that depth.)
      4. If either triggers, mark as obstacle. Collect the closest 
         obstacle point per band.
    
    Returns: 
        obstacles: list of (band_idx, rel_x, rel_z)
        floor_pts: list of (x, z) for radar debug
        obs_pts: list of (x, z) for radar debug
    """
    h, w = depth_image.shape
    band_width = w // NUM_COLUMN_BANDS
    obstacles = []
    
    debug_floor_points = []
    debug_obstacle_points = []
    
    for band_idx in range(NUM_COLUMN_BANDS):
        col = band_idx * band_width + band_width // 2
        if col >= w:
            col = w - 1
        floor_points = [] 
        
        for row in range(h - 1, -1, -SCAN_ROW_STEP):
            depth_m = depth_image[row, col] * 0.001  
            
            if depth_m < MIN_DISTANCE or depth_m > MAX_DISTANCE:
                continue
            
            x, y, z = deproject_pixel(row, col, depth_m, intrin)
            floor_points.append((row, x, y, z))
            
            if len(floor_points) >= MIN_FLOOR_PIXELS:
                break
        
        if len(floor_points) < MIN_FLOOR_PIXELS:
            continue

        z_vals = np.array([p[3] for p in floor_points])
        y_vals = np.array([p[2] for p in floor_points])
        
        z0 = z_vals[0]
        y0 = y_vals[0]
        
        dz = z_vals[-1] - z_vals[0]
        if abs(dz) > 0.05:
            floor_slope = (y_vals[-1] - y_vals[0]) / dz
        else:
            floor_slope = 0.0
        
        for fp in floor_points:
            debug_floor_points.append((fp[1], fp[3]))

        last_floor_row = floor_points[-1][0]
        last_valid_depth = floor_points[-1][3]
        
        obstacle_pixels = []
        
        for row in range(last_floor_row - SCAN_ROW_STEP, -1, -SCAN_ROW_STEP):
            depth_m = depth_image[row, col] * 0.001
            
            if depth_m < MIN_DISTANCE or depth_m > MAX_DISTANCE:
                continue
            
            x, y, z = deproject_pixel(row, col, depth_m, intrin)
            predicted_floor_y = y0 + floor_slope * (z - z0)
            height_above_floor = predicted_floor_y - y
            depth_decreased = (last_valid_depth - z) > DEPTH_JUMP_THRESHOLD
            
            if height_above_floor > OBSTACLE_HEIGHT_THRESHOLD or depth_decreased:
                obstacle_pixels.append((x, y, z))
                debug_obstacle_points.append((x, z))
            else:
                last_valid_depth = z
                if abs(z - z0) > 0.05:
                    new_slope = (y - y0) / (z - z0)
                    floor_slope = floor_slope * 0.7 + new_slope * 0.3
                debug_floor_points.append((x, z))
        
        if len(obstacle_pixels) >= MIN_OBSTACLE_PIXELS:
            obs_array = np.array(obstacle_pixels)
            z_values = obs_array[:, 2]
            k = max(1, len(obs_array) // 3)
            closest_k_idx = np.argpartition(z_values, k)[:k]
            closest_k = obs_array[closest_k_idx]
            
            rel_x = float(np.median(closest_k[:, 0]))
            rel_z = float(np.median(closest_k[:, 2]))
            
            obstacles.append((band_idx, rel_x, rel_z))
    
    return obstacles, debug_floor_points, debug_obstacle_points

while True:
    payload = {
        "localization_mode": "BLIND (ENC)",
        "aruco_pos": [],
        "vo_dx": 0.0,
        "vo_dz": 0.0,
        "vo_status": "NONE",
        "obstacles": []
    }

    if FRONT_CAMERA_SERIAL in pipelines:
        try:
            front_frames = pipelines[FRONT_CAMERA_SERIAL].wait_for_frames()

            depth_frame = front_frames.get_depth_frame()
            color_frame = front_frames.get_color_frame()
            
            if depth_frame and color_frame and depth_intrinsics is not None:
                depth_image = np.asanyarray(depth_frame.get_data())

                raw_obstacles, floor_pts, obs_pts = scan_columns_for_obstacles(
                    depth_image, depth_intrinsics
                )

                current_frame = {}
                for band_idx, rel_x, rel_z in raw_obstacles:
                    current_frame[band_idx] = (rel_x, rel_z)
                
                obstacle_history.append(current_frame)
                if len(obstacle_history) > HISTORY_FRAMES:
                    obstacle_history.pop(0)
                
                confirmed = set()
                for band_idx in current_frame:
                    hit_count = sum(1 for past in obstacle_history if band_idx in past)
                    if hit_count >= CONFIRM_FRAMES:
                        confirmed.add(band_idx)

                for band_idx in confirmed:
                    rel_x, rel_z = current_frame[band_idx]
                    payload["obstacles"].append({
                        "type": "point_cloud",
                        "rel_x": rel_x,
                        "rel_z": rel_z
                    })

                radar_size = 500
                radar_img = np.zeros((radar_size, radar_size, 3), dtype=np.uint8)
                rover_center = (radar_size // 2, radar_size - 10)
                
                grid_color = (75, 75, 75)
                text_color = (150, 150, 150)

                cv2.line(radar_img, (radar_size // 2, 0), 
                         (radar_size // 2, radar_size), grid_color, 1)
                for d_ring in np.arange(1.0, MAX_DISTANCE + 0.5, 1.0):
                    radius_px = int((d_ring / MAX_DISTANCE) * (radar_size - 20))
                    cv2.circle(radar_img, rover_center, radius_px, grid_color, 1)
                    cv2.putText(radar_img, f"{d_ring:.0f}m", 
                                (radar_size // 2 + 5, radar_size - 10 - radius_px),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1)
                cv2.circle(radar_img, rover_center, 6, (0, 255, 0), -1)

                cv2.putText(radar_img, "FLOOR SCAN", (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                for fx, fz in floor_pts:
                    px_x = int((fx + VIEW_WIDTH / 2.0) / VIEW_WIDTH * radar_size)
                    px_y = int(radar_size - 10 - (fz / MAX_DISTANCE * (radar_size - 20)))
                    if 0 <= px_x < radar_size and 0 <= px_y < radar_size:
                        cv2.circle(radar_img, (px_x, px_y), 1, (0, 80, 0), -1)

                for band_idx, rel_x, rel_z in raw_obstacles:
                    px_x = int((rel_x + VIEW_WIDTH / 2.0) / VIEW_WIDTH * radar_size)
                    px_y = int(radar_size - 10 - (rel_z / MAX_DISTANCE * (radar_size - 20)))
                    if 0 <= px_x < radar_size and 0 <= px_y < radar_size:
                        cv2.circle(radar_img, (px_x, px_y), 3, (0, 255, 255), -1)

                for band_idx in confirmed:
                    rel_x, rel_z = current_frame[band_idx]
                    px_x = int((rel_x + VIEW_WIDTH / 2.0) / VIEW_WIDTH * radar_size)
                    px_y = int(radar_size - 10 - (rel_z / MAX_DISTANCE * (radar_size - 20)))
                    if 0 <= px_x < radar_size and 0 <= px_y < radar_size:
                        cv2.circle(radar_img, (px_x, px_y), 5, (0, 0, 255), -1)

                cv2.putText(radar_img, 
                            f"Raw: {len(raw_obstacles)} | Sent: {len(confirmed)}", 
                            (10, radar_size - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
                
                color_image = np.asanyarray(color_frame.get_data())
                if not headless:
                    cv2.imshow("Front Camera RGB", color_image)
                    cv2.imshow("Top-Down Point Cloud Radar", radar_img)
                    
        except Exception as e:
            print(f"Front Camera Error: {e}")

    if USE_REAR_CAMERA and REAR_CAMERA_SERIAL in pipelines:
        try:
            rear_frames = pipelines[REAR_CAMERA_SERIAL].wait_for_frames()
            color_frame = rear_frames.get_color_frame()
            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                corners, ids, rejected = aruco_detector.detectMarkers(gray)
                if ids is not None and len(ids) > 0:
                    cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
                    
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[0], MARKER_SIZE_METERS, camera_matrix, dist_coeffs
                    )
                    tvec = tvecs[0][0]; rvec = rvecs[0][0]

                    abs_x = MARKER_GLOBAL_X + float(tvec[2])
                    abs_z = MARKER_GLOBAL_Z - float(tvec[0])

                    R, _ = cv2.Rodrigues(rvec)
                    yaw = math.atan2(R[1, 0], R[0, 0])
                    
                    yaw_corrected = yaw + math.pi / 2.0
                    while yaw_corrected > math.pi:
                        yaw_corrected -= 2 * math.pi
                    while yaw_corrected < -math.pi:
                        yaw_corrected += 2 * math.pi

                    payload["localization_mode"] = "ARUCO_LOCKED"
                    payload["aruco_pos"] = [abs_x, abs_z, float(yaw_corrected)]
                    top_left = (int(corners[0][0][0][0]), int(corners[0][0][0][1]) - 15)
                    coord_text = f"X: {abs_x:.2f}m, Z: {abs_z:.2f}m"
                    cv2.putText(color_image, coord_text, top_left, 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                    cv2.putText(color_image, coord_text, top_left, 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if not headless:
                    cv2.imshow("Rear Camera (ArUco)", color_image)

        except Exception as e:
            pass

    socket.send_string(json.dumps(payload))
    
    if not headless:
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    else:
        time.sleep(0.01)

for pipeline in pipelines.values():
    pipeline.stop()

if not headless:
    cv2.destroyAllWindows()