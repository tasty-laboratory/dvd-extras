import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

df = pd.read_csv("logs/apn/test.csv")

filter_distance = 500  # Distance in meters to filter closest points
# Only set time_window_ns once, and use correct value for 5 seconds in nanoseconds
TIME_WINDOW_SECONDS = 5
TIME_WINDOW_NS = TIME_WINDOW_SECONDS * 1e9  # 5 seconds in nanoseconds

df["z"] = -df["z"]  # Invert Z axis for correct orientation
# Separate hunter and target
hunter_df = df[df["drone"] == "hunter"].sort_values("timestamp")
target_df = df[df["drone"] == "target"].sort_values("timestamp")

# Merge on timestamp to align positions
merged = pd.merge(hunter_df, target_df, on="timestamp", suffixes=("_hunter", "_target"))

# Calculate distances at each timestamp
merged["distance"] = (
    (merged["x_hunter"] - merged["x_target"]) ** 2
    + (merged["y_hunter"] - merged["y_target"]) ** 2
    + (merged["z_hunter"] - merged["z_target"]) ** 2
) ** 0.5

# Find the closest point (minimum distance)
min_idx = merged["distance"].idxmin()
closest = merged.loc[min_idx]

# Filter merged to only points within filter distance of the closest point
cx, cy, cz = closest["x_hunter"], closest["y_hunter"], closest["z_hunter"]
merged["dist_to_closest"] = (
    (merged["x_hunter"] - cx) ** 2
    + (merged["y_hunter"] - cy) ** 2
    + (merged["z_hunter"] - cz) ** 2
) ** 0.5
print(len(merged))
filtered = merged[merged["dist_to_closest"] <= filter_distance]
print(f"Filtered points within {filter_distance}m: {len(filtered)}")
# Define all variables needed for both rows before plotting
# --- Second graph: filter by timestamp within 5s of closest point ---
closest_time = closest["timestamp"]
filtered_time = merged[
    (merged["timestamp"] >= closest_time - TIME_WINDOW_NS)
    & (merged["timestamp"] <= closest_time + TIME_WINDOW_NS)
]
print(f"Filtered points within {TIME_WINDOW_SECONDS}s: {len(filtered_time)}")

# Always center plots on the magenta spot (target at closest point)
center_x = closest["x_target"]
center_y = closest["y_target"]
center_z = closest["z_target"]

# For both spatial and temporal plots, use the same range as before but center on magenta spot
# Calculate max range for each filtered set
max_range_spatial = max(
    filtered["x_hunter"].max() - filtered["x_hunter"].min(),
    filtered["x_target"].max() - filtered["x_target"].min(),
    filtered["y_hunter"].max() - filtered["y_hunter"].min(),
    filtered["y_target"].max() - filtered["y_target"].min(),
    filtered["z_hunter"].max() - filtered["z_hunter"].min(),
    filtered["z_target"].max() - filtered["z_target"].min(),
    closest['distance'] + 1
)
max_range_temporal = max(
    filtered_time["x_hunter"].max() - filtered_time["x_hunter"].min(),
    filtered_time["x_target"].max() - filtered_time["x_target"].min(),
    filtered_time["y_hunter"].max() - filtered_time["y_hunter"].min(),
    filtered_time["y_target"].max() - filtered_time["y_target"].min(),
    filtered_time["z_hunter"].max() - filtered_time["z_hunter"].min(),
    filtered_time["z_target"].max() - filtered_time["z_target"].min(),
    closest['distance'] + 1
)

# Set limits for spatial plots
x_lim = (center_x - max_range_spatial / 2, center_x + max_range_spatial / 2)
y_lim = (center_y - max_range_spatial / 2, center_y + max_range_spatial / 2)
z_lim = (center_z - max_range_spatial / 2, center_z + max_range_spatial / 2)

# Set limits for temporal plots
x_lim2 = (center_x - max_range_temporal / 2, center_x + max_range_temporal / 2)
y_lim2 = (center_y - max_range_temporal / 2, center_y + max_range_temporal / 2)
z_lim2 = (center_z - max_range_temporal / 2, center_z + max_range_temporal / 2)

info_text = (
    f"Closest distance: {closest['distance']:.2f} m\n"
    f"ΔX: {abs(closest['x_hunter'] - closest['x_target']):.2f} m\n"
    f"ΔY: {abs(closest['y_hunter'] - closest['y_target']):.2f} m\n"
    f"ΔZ: {abs(closest['z_hunter'] - closest['z_target']):.2f} m"
)

fig, axes = plt.subplots(2, 3, figsize=(12.8, 8))

# --- Top row: spatial filter (distance) ---
axes_spatial = axes[0]
axes_spatial[0].plot(
    filtered["x_hunter"], filtered["y_hunter"], label="Hunter", color="blue"
)
axes_spatial[0].plot(
    filtered["x_target"], filtered["y_target"], label="Target", color="red"
)
axes_spatial[0].scatter(
    closest["x_hunter"], closest["y_hunter"], color="cyan", s=100, label="Closest Point"
)
axes_spatial[0].scatter(
    closest["x_target"], closest["y_target"], color="magenta", s=100
)
axes_spatial[0].set_xlabel("X")
axes_spatial[0].set_ylabel("Y")
axes_spatial[0].set_title(f"XY Plane (≤{filter_distance}m)")
axes_spatial[0].legend()
axes_spatial[0].set_xlim(x_lim)
axes_spatial[0].set_ylim(y_lim)
axes_spatial[0].text(
    0.05,
    0.95,
    info_text,
    transform=axes_spatial[0].transAxes,
    fontsize=12,
    verticalalignment="top",
    bbox=dict(facecolor="white", alpha=0.7, edgecolor="gray"),
)

axes_spatial[1].plot(
    filtered["y_hunter"], filtered["z_hunter"], label="Hunter", color="blue"
)
axes_spatial[1].plot(
    filtered["y_target"], filtered["z_target"], label="Target", color="red"
)
axes_spatial[1].scatter(
    closest["y_hunter"], closest["z_hunter"], color="cyan", s=100, label="Closest Point"
)
axes_spatial[1].scatter(
    closest["y_target"], closest["z_target"], color="magenta", s=100
)
axes_spatial[1].set_xlabel("Y")
axes_spatial[1].set_ylabel("Z")
axes_spatial[1].set_title(f"YZ Plane (≤{filter_distance}m)")
axes_spatial[1].legend()
axes_spatial[1].set_xlim(y_lim)
axes_spatial[1].set_ylim(z_lim)

axes_spatial[2].plot(
    filtered["x_hunter"], filtered["z_hunter"], label="Hunter", color="blue"
)
axes_spatial[2].plot(
    filtered["x_target"], filtered["z_target"], label="Target", color="red"
)
axes_spatial[2].scatter(
    closest["x_hunter"], closest["z_hunter"], color="cyan", s=100, label="Closest Point"
)
axes_spatial[2].scatter(
    closest["x_target"], closest["z_target"], color="magenta", s=100
)
axes_spatial[2].set_xlabel("X")
axes_spatial[2].set_ylabel("Z")
axes_spatial[2].set_title(f"XZ Plane (≤{filter_distance}m)")
axes_spatial[2].legend()
axes_spatial[2].set_xlim(x_lim)
axes_spatial[2].set_ylim(z_lim)

# --- Bottom row: temporal filter (±{TIME_WINDOW_SECONDS}s) ---
axes_temporal = axes[1]
axes_temporal[0].plot(
    filtered_time["x_hunter"], filtered_time["y_hunter"], label="Hunter", color="blue"
)
axes_temporal[0].plot(
    filtered_time["x_target"], filtered_time["y_target"], label="Target", color="red"
)
axes_temporal[0].scatter(
    closest["x_hunter"], closest["y_hunter"], color="cyan", s=100, label="Closest Point"
)
axes_temporal[0].scatter(
    closest["x_target"], closest["y_target"], color="magenta", s=100
)
axes_temporal[0].set_xlabel("X")
axes_temporal[0].set_ylabel("Y")
axes_temporal[0].set_title(f"XY Plane (±{TIME_WINDOW_SECONDS}s)")
axes_temporal[0].legend()
axes_temporal[0].set_xlim(x_lim2)
axes_temporal[0].set_ylim(y_lim2)
axes_temporal[0].text(
    0.05,
    0.95,
    info_text,
    transform=axes_temporal[0].transAxes,
    fontsize=12,
    verticalalignment="top",
    bbox=dict(facecolor="white", alpha=0.7, edgecolor="gray"),
)

axes_temporal[1].plot(
    filtered_time["y_hunter"], filtered_time["z_hunter"], label="Hunter", color="blue"
)
axes_temporal[1].plot(
    filtered_time["y_target"], filtered_time["z_target"], label="Target", color="red"
)
axes_temporal[1].scatter(
    closest["y_hunter"], closest["z_hunter"], color="cyan", s=100, label="Closest Point"
)
axes_temporal[1].scatter(
    closest["y_target"], closest["z_target"], color="magenta", s=100
)
axes_temporal[1].set_xlabel("Y")
axes_temporal[1].set_ylabel("Z")
axes_temporal[1].set_title(f"YZ Plane (±{TIME_WINDOW_SECONDS}s)")
axes_temporal[1].legend()
axes_temporal[1].set_xlim(y_lim2)
axes_temporal[1].set_ylim(z_lim2)

axes_temporal[2].plot(
    filtered_time["x_hunter"], filtered_time["z_hunter"], label="Hunter", color="blue"
)
axes_temporal[2].plot(
    filtered_time["x_target"], filtered_time["z_target"], label="Target", color="red"
)
axes_temporal[2].scatter(
    closest["x_hunter"], closest["z_hunter"], color="cyan", s=100, label="Closest Point"
)
axes_temporal[2].scatter(
    closest["x_target"], closest["z_target"], color="magenta", s=100
)
axes_temporal[2].set_xlabel("X")
axes_temporal[2].set_ylabel("Z")
axes_temporal[2].set_title(f"XZ Plane (±{TIME_WINDOW_SECONDS}s)")
axes_temporal[2].legend()
axes_temporal[2].set_xlim(x_lim2)
axes_temporal[2].set_ylim(z_lim2)

plt.suptitle(
    f"Drone Paths and Closest Encounter Point\nTop: ≤{filter_distance}m | Bottom: ±{TIME_WINDOW_SECONDS}s"
)
plt.tight_layout()

print(f"Closest point distance: {closest['distance']:.3f} m")
print(f"Delta X: {abs(closest['x_hunter'] - closest['x_target']):.3f} m")
print(f"Delta Y: {abs(closest['y_hunter'] - closest['y_target']):.3f} m")
print(f"Delta Z: {abs(closest['z_hunter'] - closest['z_target']):.3f} m")

# --- Second graph: filter by timestamp within 5s of closest point ---
closest_time = closest["timestamp"]
filtered_time = merged[
    (merged["timestamp"] >= closest_time - TIME_WINDOW_NS)
    & (merged["timestamp"] <= closest_time + TIME_WINDOW_NS)
]

# fig2, axes2 = plt.subplots(1, 3, figsize=(18, 6))

# # Calculate global min/max for x, y, z in filtered_time data
# x_min2 = min(filtered_time["x_hunter"].min(), filtered_time["x_target"].min())
# x_max2 = max(filtered_time["x_hunter"].max(), filtered_time["x_target"].max())
# y_min2 = min(filtered_time["y_hunter"].min(), filtered_time["y_target"].min())
# y_max2 = max(filtered_time["y_hunter"].max(), filtered_time["y_target"].max())
# z_min2 = min(filtered_time["z_hunter"].min(), filtered_time["z_target"].min())
# z_max2 = max(filtered_time["z_hunter"].max(), filtered_time["z_target"].max())
# ranges2 = [x_max2 - x_min2, y_max2 - y_min2, z_max2 - z_min2]
# max_range2 = max(ranges2)
# x_center2 = (x_max2 + x_min2) / 2
# y_center2 = (y_max2 + y_min2) / 2
# z_center2 = (z_max2 + z_min2) / 2
# x_lim2 = (x_center2 - max_range2 / 2, x_center2 + max_range2 / 2)
# y_lim2 = (y_center2 - max_range2 / 2, y_center2 + max_range2 / 2)
# z_lim2 = (z_center2 - max_range2 / 2, z_center2 + max_range2 / 2)

# # XY plot
# axes2[0].plot(
#     filtered_time["x_hunter"], filtered_time["y_hunter"], label="Hunter", color="blue"
# )
# axes2[0].plot(
#     filtered_time["x_target"], filtered_time["y_target"], label="Target", color="red"
# )
# axes2[0].scatter(
#     closest["x_hunter"], closest["y_hunter"], color="cyan", s=100, label="Closest Point"
# )
# axes2[0].scatter(closest["x_target"], closest["y_target"], color="magenta", s=100)
# axes2[0].set_xlabel("X")
# axes2[0].set_ylabel("Y")
# axes2[0].set_title("XY Plane (±{TIME_WINDOW_SECONDS}s)")
# axes2[0].legend()
# axes2[0].set_xlim(x_lim2)
# axes2[0].set_ylim(y_lim2)

# # Print closest point info on the graph
# axes2[0].text(
#     0.05,
#     0.95,
#     info_text,
#     transform=axes2[0].transAxes,
#     fontsize=12,
#     verticalalignment="top",
#     bbox=dict(facecolor="white", alpha=0.7, edgecolor="gray"),
# )

# # YZ plot
# axes2[1].plot(
#     filtered_time["y_hunter"], filtered_time["z_hunter"], label="Hunter", color="blue"
# )
# axes2[1].plot(
#     filtered_time["y_target"], filtered_time["z_target"], label="Target", color="red"
# )
# axes2[1].scatter(
#     closest["y_hunter"], closest["z_hunter"], color="cyan", s=100, label="Closest Point"
# )
# axes2[1].scatter(closest["y_target"], closest["z_target"], color="magenta", s=100)
# axes2[1].set_xlabel("Y")
# axes2[1].set_ylabel("Z")
# axes2[1].set_title("YZ Plane (±{TIME_WINDOW_SECONDS}s)")
# axes2[1].legend()
# axes2[1].set_xlim(y_lim2)
# axes2[1].set_ylim(z_lim2)

# # XZ plot
# axes2[2].plot(
#     filtered_time["x_hunter"], filtered_time["z_hunter"], label="Hunter", color="blue"
# )
# axes2[2].plot(
#     filtered_time["x_target"], filtered_time["z_target"], label="Target", color="red"
# )
# axes2[2].scatter(
#     closest["x_hunter"], closest["z_hunter"], color="cyan", s=100, label="Closest Point"
# )
# axes2[2].scatter(closest["x_target"], closest["z_target"], color="magenta", s=100)
# axes2[2].set_xlabel("X")
# axes2[2].set_ylabel("Z")
# axes2[2].set_title("XZ Plane (±{TIME_WINDOW_SECONDS}s)")
# axes2[2].legend()
# axes2[2].set_xlim(x_lim2)
# axes2[2].set_ylim(z_lim2)

# 3D
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# hunter_df = df[df["drone"] == "hunter"]
# target_df = df[df["drone"] == "target"]

# # Plot hunter with time as color
# sc1 = ax.scatter(hunter_df['x'], hunter_df['y'], -hunter_df['z'], c=hunter_df['timestamp'], cmap='Blues', label='Hunter', s=5)
# # Plot target with time as color
# sc2 = ax.scatter(target_df['x'], target_df['y'], -target_df['z'], c=target_df['timestamp'], cmap='Reds', label='Target', s=5)

# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.legend()
# ax.text2D(0.05, 0.95, "Hunter and Target Positions Over Time, Time is Light to Dark", transform=ax.transAxes)
# ax.set_box_aspect([1,1,1])
# minX = min(hunter_df['x'])
# maxX = max(hunter_df['x'])
# minY = min(hunter_df['y'])
# maxY = max(hunter_df['y'])
# meanY = (minY + maxY)/2
# minZ = -min(hunter_df['z'])
# maxZ = -max(hunter_df['z'])
# meanZ = (minZ + maxZ)/2
# delta = maxX - minX
# ax.set_xlim(minX,maxX)
# ax.set_ylim(meanY - delta / 2,meanY + delta / 2)
# ax.set_zlim(meanZ - delta / 2,meanZ + delta / 2)
plt.show()
