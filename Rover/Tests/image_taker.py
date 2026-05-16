import pyrealsense2 as rs
import cv2
import numpy as np
import os
import time


# =========================
# USER SETTINGS
# =========================

REAR_CAMERA_SERIAL = "247122073398"
SAVE_DIR = "calibration_images"

WIDTH = 640
HEIGHT = 480
FPS = 30


# =========================
# SETUP
# =========================

os.makedirs(SAVE_DIR, exist_ok=True)

pipeline = rs.pipeline()
config = rs.config()

config.enable_device(REAR_CAMERA_SERIAL)
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

print("Starting RealSense camera...")
pipeline.start(config)

img_count = 0

print()
print("==============================================")
print(" Camera Calibration Image Capture")
print("==============================================")
print("Controls:")
print("  SPACE = save current image")
print("  q     = quit")
print("==============================================")
print()
print(f"Saving images to: {SAVE_DIR}")
print()


# =========================
# MAIN LOOP
# =========================

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        img = np.asanyarray(color_frame.get_data())

        display = img.copy()

        cv2.rectangle(display, (0, 0), (WIDTH, 70), (0, 0, 0), -1)

        cv2.putText(
            display,
            f"Saved images: {img_count}",
            (20, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.putText(
            display,
            "SPACE = save | q = quit",
            (20, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.imshow("Calibration Image Capture", display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord(" "):
            filename = os.path.join(SAVE_DIR, f"calib_{img_count:03d}.png")
            cv2.imwrite(filename, img)
            print(f"Saved: {filename}")
            img_count += 1
            time.sleep(0.2)

        elif key == ord("q"):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print()
    print(f"Done. Saved {img_count} images.")