import pandas as pd
import numpy as np


INPUT_CSV = "aruco_accuracy_test.csv"
OUTPUT_CSV = "aruco_summary_stats.csv"

# Arena ground truth:
# +Z is out from the ArUco marker
# +X is left/right of the ArUco marker
GROUND_TRUTH_X_M = 2.57
GROUND_TRUTH_Y_M = 0.0

# If measured_x_m comes out with the opposite sign, change this to -1.0
X_SIGN = 1.0


def angle_error_deg(measured_deg, truth_deg):
    return (measured_deg - truth_deg + 180.0) % 360.0 - 180.0


def camera_tvec_to_arena(row):
    """
    Convert raw OpenCV camera-frame tvec into arena-frame x/z.

    OpenCV camera frame:
      tvec_x = right in image
      tvec_y = down in image
      tvec_z = forward from camera toward marker

    Arena frame:
      +Z = out from ArUco marker
      +X = left/right of ArUco marker

    test_angle_deg is measured counterclockwise from straight-on to the ArUco.
    """
    x_cam = row["tvec_x_m"]
    y_cam = row["tvec_y_m"]
    z_cam = row["tvec_z_m"]

    theta = np.deg2rad(row["test_angle_deg"])

    # At 0 deg:
    #   arena_z ≈ tvec_z
    #   arena_x ≈ -tvec_x
    #
    # For nonzero test angle, rotate the camera-frame horizontal vector
    # into the arena frame using the known ground-truth test angle.
    arena_x = z_cam * np.sin(theta) - x_cam * np.cos(theta)
    arena_z = z_cam * np.cos(theta) + x_cam * np.sin(theta)

    # OpenCV +y is down. Arena +y is up.
    arena_y = -y_cam

    arena_x *= X_SIGN

    return pd.Series({
        "measured_x_m": arena_x,
        "measured_y_m": arena_y,
        "measured_z_m": arena_z,
    })


df = pd.read_csv(INPUT_CSV)

# Convert raw camera-frame tvec into arena-frame position
arena_xyz = df.apply(camera_tvec_to_arena, axis=1)
df["measured_x_m"] = arena_xyz["measured_x_m"]
df["measured_y_m"] = arena_xyz["measured_y_m"]
df["measured_z_m"] = arena_xyz["measured_z_m"]

# Ground truth arena position
df["ground_truth_x_m"] = GROUND_TRUTH_X_M
df["ground_truth_y_m"] = GROUND_TRUTH_Y_M
df["ground_truth_z_m"] = df["test_distance_m"]

# Ground truth straight-line range from marker to camera
df["ground_truth_range_m"] = np.sqrt(
    df["ground_truth_x_m"]**2 +
    df["ground_truth_y_m"]**2 +
    df["ground_truth_z_m"]**2
)

# Measured straight-line range from transformed arena x/y/z
df["measured_range_m"] = np.sqrt(
    df["measured_x_m"]**2 +
    df["measured_y_m"]**2 +
    df["measured_z_m"]**2
)

# Error columns
df["x_error_m"] = df["measured_x_m"] - df["ground_truth_x_m"]
df["y_error_m"] = df["measured_y_m"] - df["ground_truth_y_m"]
df["z_error_m"] = df["measured_z_m"] - df["ground_truth_z_m"]
df["range_error_m"] = df["measured_range_m"] - df["ground_truth_range_m"]

# Keep yaw analysis if measured_yaw_deg exists in your CSV,
# but remember this is ArUco yaw, not IMU/ground-truth yaw.
if "measured_yaw_deg" in df.columns:
    df["yaw_error_deg"] = angle_error_deg(
        df["measured_yaw_deg"],
        df["test_angle_deg"]
    )
else:
    df["measured_yaw_deg"] = np.nan
    df["yaw_error_deg"] = np.nan


group_cols = ["test_distance_m", "test_angle_deg"]

summary = df.groupby(group_cols).agg(
    samples=("sample_index", "count"),

    mean_x_m=("measured_x_m", "mean"),
    std_x_m=("measured_x_m", "std"),
    mean_x_error_m=("x_error_m", "mean"),
    std_x_error_m=("x_error_m", "std"),

    mean_y_m=("measured_y_m", "mean"),
    std_y_m=("measured_y_m", "std"),
    mean_y_error_m=("y_error_m", "mean"),
    std_y_error_m=("y_error_m", "std"),

    mean_z_m=("measured_z_m", "mean"),
    std_z_m=("measured_z_m", "std"),
    mean_z_error_m=("z_error_m", "mean"),
    std_z_error_m=("z_error_m", "std"),

    mean_range_m=("measured_range_m", "mean"),
    std_range_m=("measured_range_m", "std"),
    mean_range_error_m=("range_error_m", "mean"),
    std_range_error_m=("range_error_m", "std"),

    ground_truth_x_m=("ground_truth_x_m", "mean"),
    ground_truth_z_m=("ground_truth_z_m", "mean"),
    ground_truth_range_m=("ground_truth_range_m", "mean"),

    mean_yaw_deg=("measured_yaw_deg", "mean"),
    std_yaw_deg=("measured_yaw_deg", "std"),
    mean_yaw_error_deg=("yaw_error_deg", "mean"),
    std_yaw_error_deg=("yaw_error_deg", "std"),
).reset_index()

summary["abs_mean_x_error_m"] = summary["mean_x_error_m"].abs()
summary["abs_mean_y_error_m"] = summary["mean_y_error_m"].abs()
summary["abs_mean_z_error_m"] = summary["mean_z_error_m"].abs()
summary["abs_mean_range_error_m"] = summary["mean_range_error_m"].abs()
summary["abs_mean_yaw_error_deg"] = summary["mean_yaw_error_deg"].abs()

pd.set_option("display.max_columns", None)
pd.set_option("display.width", 240)

print("\n===== ArUco Arena-Frame X/Y/Z/Range/Yaw Summary =====\n")
print(summary)

print("\n===== X and Z Error Only =====\n")
print(summary[[
    "test_distance_m",
    "test_angle_deg",
    "samples",

    "ground_truth_x_m",
    "mean_x_m",
    "std_x_m",
    "mean_x_error_m",

    "ground_truth_z_m",
    "mean_z_m",
    "std_z_m",
    "mean_z_error_m",
]])

print("\n===== Range Error Only =====\n")
print(summary[[
    "test_distance_m",
    "test_angle_deg",
    "samples",
    "ground_truth_range_m",
    "mean_range_m",
    "std_range_m",
    "mean_range_error_m",
]])

print("\n===== Yaw Error Only =====\n")
print(summary[[
    "test_distance_m",
    "test_angle_deg",
    "samples",
    "mean_yaw_deg",
    "std_yaw_deg",
    "mean_yaw_error_deg",
]])

summary.to_csv(OUTPUT_CSV, index=False)

print(f"\nSaved summary statistics to: {OUTPUT_CSV}")