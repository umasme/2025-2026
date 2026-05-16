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

MARKER_SIZE_METERS = 0.20
SAMPLES_PER_TEST = 250

OUTPUT_CSV = "aruco_accuracy_test.csv"

# Test matrix: distance in meters, camera/rover yaw angle relative to marker
TEST_DISTANCES_M = [1.0, 2.0, 4.0, 6.0]
TEST_ANGLES_DEG = [0, 30, 60]

# If 1 m head-on / 30 deg is physically not possible, press x during that test
# and the script will move to the next one.

# Camera intrinsics from your original script.
# For better results, replace these with calibrated intrinsics if available.
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

def wrap_to_pi(angle_rad):
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def rvec_to_yaw(rvec):
    """
    Converts OpenCV ArUco rotation vector to yaw.

    This follows the same basic method from your original script:
        R, _ = cv2.Rodrigues(rvec)
        yaw = atan2(R[1, 0], R[0, 0])

    Then applies the same +90 deg correction you were using.
    """
    R, _ = cv2.Rodrigues(rvec)

    yaw_rad = math.atan2(R[1, 0], R[0, 0])

    yaw_corrected_rad = yaw_rad + math.pi / 2.0
    yaw_corrected_rad = wrap_to_pi(yaw_corrected_rad)

    return yaw_rad, yaw_corrected_rad


def draw_text_box(img, lines, position="top_right"):
    """
    Draws readable text on the OpenCV image.
    position options:
      "top_left"
      "top_right"
    """
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.65
    thickness = 2
    line_height = 28
    padding = 10

    img_h, img_w = img.shape[:2]

    box_width = 600
    box_height = 20 + line_height * len(lines)

    if position == "top_right":
        x = img_w - box_width - padding
        y = 30
    else:
        x = 20
        y = 30

    cv2.rectangle(
        img,
        (x - padding, y - 25),
        (x - padding + box_width, y - 25 + box_height),
        (0, 0, 0),
        -1
    )

    for i, line in enumerate(lines):
        cv2.putText(
            img,
            line,
            (x, y + i * line_height),
            font,
            font_scale,
            (0, 255, 0),
            thickness
        )


def write_csv_header(writer):
    writer.writerow([
        "timestamp_s",
        "datetime",
        "trial_number",
        "test_distance_m",
        "test_angle_deg",
        "sample_index",

        "marker_id",

        "measured_x_m",
        "measured_y_m",
        "measured_distance_m",
        "measured_yaw_rad",
        "measured_yaw_deg",

        "raw_yaw_rad",
        "raw_yaw_deg",

        "tvec_x_m",
        "tvec_y_m",
        "tvec_z_m",

        "rvec_x",
        "rvec_y",
        "rvec_z",

        "corner_0_x_px",
        "corner_0_y_px",
        "corner_1_x_px",
        "corner_1_y_px",
        "corner_2_x_px",
        "corner_2_y_px",
        "corner_3_x_px",
        "corner_3_y_px",
    ])


def write_detection_row(
    writer,
    trial_number,
    test_distance_m,
    test_angle_deg,
    sample_index,
    marker_id,
    tvec,
    rvec,
    corners
):
    timestamp_s = time.time()
    datetime_str = datetime.now().isoformat(timespec="milliseconds")

    tvec_x = float(tvec[0])
    tvec_y = float(tvec[1])
    tvec_z = float(tvec[2])

    rvec_x = float(rvec[0])
    rvec_y = float(rvec[1])
    rvec_z = float(rvec[2])

    measured_x_m = tvec_x
    measured_y_m = tvec_y
    measured_distance_m = math.sqrt(tvec_x**2 + tvec_y**2 + tvec_z**2)

    raw_yaw_rad, measured_yaw_rad = rvec_to_yaw(rvec)
    raw_yaw_deg = math.degrees(raw_yaw_rad)
    measured_yaw_deg = math.degrees(measured_yaw_rad)

    # corners shape for one marker: (4, 2)
    c = corners.reshape((4, 2))

    writer.writerow([
        timestamp_s,
        datetime_str,
        trial_number,
        test_distance_m,
        test_angle_deg,
        sample_index,

        int(marker_id),

        measured_x_m,
        measured_y_m,
        measured_distance_m,
        measured_yaw_rad,
        measured_yaw_deg,

        raw_yaw_rad,
        raw_yaw_deg,

        tvec_x,
        tvec_y,
        tvec_z,

        rvec_x,
        rvec_y,
        rvec_z,

        float(c[0][0]),
        float(c[0][1]),
        float(c[1][0]),
        float(c[1][1]),
        float(c[2][0]),
        float(c[2][1]),
        float(c[3][0]),
        float(c[3][1]),
    ])


# =========================
# MAIN PROGRAM
# =========================

def main():
    print("==============================================")
    print("        ArUco Accuracy CSV Logging Test        ")
    print("==============================================")
    print(f"Rear camera serial: {REAR_CAMERA_SERIAL}")
    print(f"Marker size: {MARKER_SIZE_METERS:.3f} m")
    print(f"Samples per test: {SAMPLES_PER_TEST}")
    print(f"Output CSV: {OUTPUT_CSV}")
    print()
    print("Controls:")
    print("  SPACE = start current test condition")
    print("  x     = skip/cancel current test condition")
    print("  q     = quit entire script")
    print("==============================================")
    print()

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_device(REAR_CAMERA_SERIAL)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    try:
        print("Starting RealSense rear camera...")
        pipeline.start(config)
        print("Camera started.")
    except Exception as e:
        print(f"CRITICAL ERROR: Could not start camera: {e}")
        return

    trial_number = 0

    try:
        with open(OUTPUT_CSV, mode="w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            write_csv_header(writer)

            for test_distance_m in TEST_DISTANCES_M:
                for test_angle_deg in TEST_ANGLES_DEG:
                    trial_number += 1
                    sample_index = 0
                    waiting_to_start = True
                    test_skipped = False

                    print()
                    print("==============================================")
                    print(f"Trial {trial_number}")
                    print(f"Set rover/camera to distance: {test_distance_m:.1f} m")
                    print(f"Set rover/camera angle to:    {test_angle_deg} deg")
                    print("Press SPACE to start logging this condition.")
                    print("Press x to skip this condition.")
                    print("Press q to quit.")
                    print("==============================================")

                    while waiting_to_start:
                        frames = pipeline.wait_for_frames()
                        color_frame = frames.get_color_frame()

                        if not color_frame:
                            continue

                        color_image = np.asanyarray(color_frame.get_data())

                        lines = [
                            f"Trial {trial_number}",
                            f"Set distance: {test_distance_m:.1f} m",
                            f"Set angle: {test_angle_deg} deg",
                            "SPACE = start logging",
                            "x = skip this condition",
                            "q = quit"
                        ]
                        draw_text_box(color_image, lines)

                        cv2.imshow("ArUco Accuracy Logger", color_image)
                        key = cv2.waitKey(10) & 0xFF

                        if key == ord(" "):
                            waiting_to_start = False
                            print("Logging started...")
                        elif key == ord("x"):
                            waiting_to_start = False
                            test_skipped = True
                            print("Test condition skipped.")
                        elif key == ord("q"):
                            print("Quitting.")
                            return

                    if test_skipped:
                        continue

                    while sample_index < SAMPLES_PER_TEST:
                        frames = pipeline.wait_for_frames()
                        color_frame = frames.get_color_frame()

                        if not color_frame:
                            continue

                        color_image = np.asanyarray(color_frame.get_data())
                        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

                        corners, ids, rejected = aruco_detector.detectMarkers(gray)

                        detected_this_frame = False

                        if ids is not None and len(ids) > 0:
                            cv2.aruco.drawDetectedMarkers(color_image, corners, ids)

                            # Use the first detected marker.
                            # If you only have one marker in the scene, this is fine.
                            marker_corners = corners[0]
                            marker_id = ids[0][0]

                            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                                marker_corners,
                                MARKER_SIZE_METERS,
                                camera_matrix,
                                dist_coeffs
                            )

                            rvec = rvecs[0][0]
                            tvec = tvecs[0][0]

                            write_detection_row(
                                writer=writer,
                                trial_number=trial_number,
                                test_distance_m=test_distance_m,
                                test_angle_deg=test_angle_deg,
                                sample_index=sample_index,
                                marker_id=marker_id,
                                tvec=tvec,
                                rvec=rvec,
                                corners=marker_corners
                            )

                            sample_index += 1
                            detected_this_frame = True

                            # Draw axes if available in your OpenCV build
                            try:
                                cv2.drawFrameAxes(
                                    color_image,
                                    camera_matrix,
                                    dist_coeffs,
                                    rvec,
                                    tvec,
                                    MARKER_SIZE_METERS * 0.5
                                )
                            except Exception:
                                pass

                            measured_distance_m = math.sqrt(
                                float(tvec[0])**2 +
                                float(tvec[1])**2 +
                                float(tvec[2])**2
                            )

                            raw_yaw_rad, measured_yaw_rad = rvec_to_yaw(rvec)

                            status_lines = [
                                f"Trial {trial_number}: {test_distance_m:.1f} m, {test_angle_deg} deg",
                                f"Logging sample {sample_index}/{SAMPLES_PER_TEST}",
                                f"Detected marker ID: {int(marker_id)}",
                                f"Measured x: {float(tvec[0]):.3f} m",
                                f"Measured y: {float(tvec[1]):.3f} m",
                                f"Measured distance: {measured_distance_m:.3f} m",
                                f"Measured yaw: {math.degrees(measured_yaw_rad):.2f} deg",
                                "x = skip condition, q = quit"
                            ]

                        else:
                            status_lines = [
                                f"Trial {trial_number}: {test_distance_m:.1f} m, {test_angle_deg} deg",
                                f"Waiting for detection...",
                                f"Valid samples: {sample_index}/{SAMPLES_PER_TEST}",
                                "x = skip condition if marker cannot be detected",
                                "q = quit"
                            ]

                        draw_text_box(color_image, status_lines)

                        if detected_this_frame:
                            cv2.circle(color_image, (20, 455), 8, (0, 255, 0), -1)
                            cv2.putText(
                                color_image,
                                "LOGGED",
                                (40, 462),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (0, 255, 0),
                                2
                            )
                        else:
                            cv2.circle(color_image, (20, 455), 8, (0, 0, 255), -1)
                            cv2.putText(
                                color_image,
                                "NO DETECTION",
                                (40, 462),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (0, 0, 255),
                                2
                            )

                        cv2.imshow("ArUco Accuracy Logger", color_image)
                        key = cv2.waitKey(1) & 0xFF

                        if key == ord("x"):
                            print(f"Trial {trial_number} skipped early at {sample_index} samples.")
                            break
                        elif key == ord("q"):
                            print("Quitting.")
                            return

                    csv_file.flush()

                    if sample_index >= SAMPLES_PER_TEST:
                        print(
                            f"Trial {trial_number} complete: "
                            f"{test_distance_m:.1f} m, {test_angle_deg} deg, "
                            f"{SAMPLES_PER_TEST} samples logged."
                        )

            print()
            print("==============================================")
            print("All test conditions complete.")
            print(f"Data saved to: {OUTPUT_CSV}")
            print("==============================================")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()