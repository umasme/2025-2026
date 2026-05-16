import cv2
import numpy as np
import glob
import os

# Number of INNER corners
CHECKERBOARD = (8, 6)

# Size of one checkerboard square in meters
SQUARE_SIZE_M = 0.0284  # 25 mm, change this to your actual square size

IMAGE_DIR = "calibration_images"
OUTPUT_FILE = "camera_calibration.npz"

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE_M

objpoints = []
imgpoints = []

images = glob.glob(os.path.join(IMAGE_DIR, "*.png"))

if len(images) == 0:
    raise RuntimeError("No calibration images found.")

gray_shape = None

for filename in images:
    img = cv2.imread(filename)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_shape = gray.shape[::-1]

    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if found:
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001
        )

        corners_refined = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            criteria
        )

        objpoints.append(objp)
        imgpoints.append(corners_refined)

        cv2.drawChessboardCorners(img, CHECKERBOARD, corners_refined, found)
        cv2.imshow("Detected corners", img)
        cv2.waitKey(100)

        print(f"Used: {filename}")
    else:
        print(f"Skipped, corners not found: {filename}")

cv2.destroyAllWindows()

if len(objpoints) < 10:
    raise RuntimeError(
        f"Only {len(objpoints)} valid calibration images. "
        "Use at least 10, preferably 20-40."
    )

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    gray_shape,
    None,
    None
)

print()
print("Calibration complete")
print(f"RMS reprojection error: {ret}")
print()
print("Camera matrix:")
print(camera_matrix)
print()
print("Distortion coefficients:")
print(dist_coeffs.ravel())

np.savez(
    OUTPUT_FILE,
    camera_matrix=camera_matrix,
    dist_coeffs=dist_coeffs,
    rms_error=ret
)

print(f"\nSaved calibration to {OUTPUT_FILE}")