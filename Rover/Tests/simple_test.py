import pyrealsense2 as rs
import numpy as np
import cv2
import csv
import math
import time
from datetime import datetime


# =========================
# USER SETTINGS
# =========================

REAR_CAMERA_SERIAL = "247122073398"

MARKER_SIZE_METERS = 0.175
SAMPLES_PER_TEST = 250

OUTPUT_CSV = "aruco_simple_distance_test.csv"

# Simple sanity tests: marker centered, head-on
TESTS = [
    {"name": "Test_A_2m_head_on", "ground_truth_z_m": 2.0},
    {"name": "Test_B_4m_head_on", "ground_truth_z_m": 4.0},
]

camera_matrix = np.array([
    [616.7609796,   0.0, 310.24420251],
    [  0.0, 619.9531825, 235.66156656],
    [  0.0,   0.0,   1.0]
], dtype=np.float32)

dist_coeffs = np.zeros((4, 1), dtype=np.float32)


# =========================
# ARUCO SETUP
# =========================

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)


# =========================
# HELPER FUNCTIONS
# =========================

def estimate_pose_single_marker(marker_corners):
    if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            marker_corners,
            MARKER_SIZE_METERS,
            camera_matrix,
            dist_coeffs
        )
        return rvecs[0][0], tvecs[0][0]

    # Fallback if estimatePoseSingleMarkers is unavailable
    half = MARKER_SIZE_METERS / 2.0

    obj_points = np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float32)

    img_points = marker_corners.reshape((4, 2)).astype(np.float32)

    success, rvec, tvec = cv2.solvePnP(
        obj_points,
        img_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE
    )

    if not success:
        raise RuntimeError("solvePnP failed")

    return rvec.reshape(3), tvec.reshape(3)


def draw_text(img, lines):
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.65
    thickness = 2
    line_height = 28

    cv2.rectangle(img, (10, 10), (630, 10 + 30 * len(lines)), (0, 0, 0), -1)

    for i, line in enumerate(lines):
        cv2.putText(
            img,
            line,
            (20, 38 + i * line_height),
            font,
            font_scale,
            (0, 255, 0),
            thickness
        )


def write_header(writer):
    writer.writerow([
        "timestamp_s",
        "datetime",
        "test_name",
        "ground_truth_z_m",
        "sample_index",
        "marker_id",

        "tvec_x_m",
        "tvec_y_m",
        "tvec_z_m",
        "range_m",

        "x_error_m",
        "z_error_m",
        "range_error_m",
    ])


def write_row(writer, test_name, ground_truth_z_m, sample_index, marker_id, tvec):
    timestamp_s = time.time()
    datetime_str = datetime.now().isoformat(timespec="milliseconds")

    x = float(tvec[0])
    y = float(tvec[1])
    z = float(tvec[2])

    range_m = math.sqrt(x**2 + y**2 + z**2)

    # For head-on centered tests:
    # ground truth x = 0
    # ground truth z = 2 or 4
    # ground truth range ≈ ground_truth_z_m
    x_error_m = x - 0.0
    z_error_m = z - ground_truth_z_m
    range_error_m = range_m - ground_truth_z_m

    writer.writerow([
        timestamp_s,
        datetime_str,
        test_name,
        ground_truth_z_m,
        sample_index,
        int(marker_id),

        x,
        y,
        z,
        range_m,

        x_error_m,
        z_error_m,
        range_error_m,
    ])


# =========================
# MAIN
# =========================

def main():
    print("==============================================")
    print(" Simple ArUco Head-On Distance Test")
    print("==============================================")
    print("Tests:")
    print("  Test A: x = 0, z = 2 m, head-on, marker centered")
    print("  Test B: x = 0, z = 4 m, head-on, marker centered")
    print()
    print("Controls:")
    print("  SPACE = start current test")
    print("  x     = skip current test")
    print("  q     = quit")
    print("==============================================")

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_device(REAR_CAMERA_SERIAL)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    try:
        print("Starting RealSense camera...")
        pipeline.start(config)
        print("Camera started.")
    except Exception as e:
        print(f"CRITICAL ERROR: Could not start camera: {e}")
        return

    try:
        with open(OUTPUT_CSV, mode="w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            write_header(writer)

            for test in TESTS:
                test_name = test["name"]
                ground_truth_z_m = test["ground_truth_z_m"]

                sample_index = 0
                waiting_to_start = True
                skip_test = False

                print()
                print("==============================================")
                print(test_name)
                print(f"Place camera/rover at x = 0, z = {ground_truth_z_m:.1f} m")
                print("Camera should be head-on to marker.")
                print("Marker should be centered in image.")
                print("Press SPACE to start.")
                print("Press x to skip.")
                print("Press q to quit.")
                print("==============================================")

                while waiting_to_start:
                    frames = pipeline.wait_for_frames()
                    color_frame = frames.get_color_frame()
                    if not color_frame:
                        continue

                    img = np.asanyarray(color_frame.get_data())

                    draw_text(img, [
                        test_name,
                        f"Set x = 0, z = {ground_truth_z_m:.1f} m",
                        "Head-on, marker centered",
                        "SPACE = start | x = skip | q = quit"
                    ])

                    cv2.imshow("Simple ArUco Distance Test", img)
                    key = cv2.waitKey(10) & 0xFF

                    if key == ord(" "):
                        waiting_to_start = False
                    elif key == ord("x"):
                        waiting_to_start = False
                        skip_test = True
                    elif key == ord("q"):
                        return

                if skip_test:
                    print(f"Skipped {test_name}")
                    continue

                while sample_index < SAMPLES_PER_TEST:
                    frames = pipeline.wait_for_frames()
                    color_frame = frames.get_color_frame()
                    if not color_frame:
                        continue

                    img = np.asanyarray(color_frame.get_data())
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                    corners, ids, rejected = aruco_detector.detectMarkers(gray)

                    detected = False

                    if ids is not None and len(ids) > 0:
                        marker_corners = corners[0]
                        marker_id = ids[0][0]

                        cv2.aruco.drawDetectedMarkers(img, corners, ids)

                        try:
                            rvec, tvec = estimate_pose_single_marker(marker_corners)
                        except Exception as e:
                            print(f"Pose estimate failed: {e}")
                            continue

                        write_row(
                            writer,
                            test_name,
                            ground_truth_z_m,
                            sample_index,
                            marker_id,
                            tvec
                        )

                        x = float(tvec[0])
                        y = float(tvec[1])
                        z = float(tvec[2])
                        range_m = math.sqrt(x**2 + y**2 + z**2)

                        sample_index += 1
                        detected = True

                        try:
                            cv2.drawFrameAxes(
                                img,
                                camera_matrix,
                                dist_coeffs,
                                rvec,
                                tvec,
                                MARKER_SIZE_METERS * 0.5
                            )
                        except Exception:
                            pass

                        draw_text(img, [
                            test_name,
                            f"Sample {sample_index}/{SAMPLES_PER_TEST}",
                            f"tvec_x: {x:.3f} m",
                            f"tvec_y: {y:.3f} m",
                            f"tvec_z: {z:.3f} m",
                            f"range:  {range_m:.3f} m",
                            f"z error: {z - ground_truth_z_m:+.3f} m",
                            "x = skip | q = quit"
                        ])

                    else:
                        draw_text(img, [
                            test_name,
                            f"Waiting for marker...",
                            f"Samples: {sample_index}/{SAMPLES_PER_TEST}",
                            "x = skip | q = quit"
                        ])

                    if detected:
                        cv2.circle(img, (20, 455), 8, (0, 255, 0), -1)
                        cv2.putText(img, "LOGGED", (40, 462),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0, 255, 0), 2)
                    else:
                        cv2.circle(img, (20, 455), 8, (0, 0, 255), -1)
                        cv2.putText(img, "NO DETECTION", (40, 462),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0, 0, 255), 2)

                    cv2.imshow("Simple ArUco Distance Test", img)
                    key = cv2.waitKey(1) & 0xFF

                    if key == ord("x"):
                        print(f"Skipped {test_name} early at {sample_index} samples.")
                        break
                    elif key == ord("q"):
                        return

                csv_file.flush()
                print(f"Finished {test_name}: {sample_index} samples")

        print()
        print("==============================================")
        print(f"Saved data to: {OUTPUT_CSV}")
        print("==============================================")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()