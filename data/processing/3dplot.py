import pandas as pd
import numpy as np
import plotly.graph_objects as go

# Load data
df = pd.read_csv("logs/pn/18.csv")
df["z"] = -df["z"]  # same as your script

hunter_df = df[df["drone"] == "hunter"].sort_values("timestamp")
target_df = df[df["drone"] == "target"].sort_values("timestamp")

# Timestamp-aligned merge (same as your logic)
merged = pd.merge(
    hunter_df,
    target_df,
    on="timestamp",
    suffixes=("_hunter", "_target")
)

# Distance calculation
merged["distance"] = np.sqrt(
    (merged["x_hunter"] - merged["x_target"])**2 +
    (merged["y_hunter"] - merged["y_target"])**2 +
    (merged["z_hunter"] - merged["z_target"])**2
)

# Closest encounter
closest = merged.loc[merged["distance"].idxmin()]

# ------------------ Plotly Figure ------------------

fig = go.Figure()

# Hunter path
fig.add_trace(go.Scatter3d(
    x=hunter_df["x"],
    y=hunter_df["y"],
    z=hunter_df["z"],
    mode="lines",
    name="Hunter",
    line=dict(width=4)
))

# Target path
fig.add_trace(go.Scatter3d(
    x=target_df["x"],
    y=target_df["y"],
    z=target_df["z"],
    mode="lines",
    name="Target",
    line=dict(width=4)
))

# Closest points
fig.add_trace(go.Scatter3d(
    x=[closest["x_hunter"]],
    y=[closest["y_hunter"]],
    z=[closest["z_hunter"]],
    mode="markers",
    marker=dict(size=7),
    name="Hunter @ closest"
))

fig.add_trace(go.Scatter3d(
    x=[closest["x_target"]],
    y=[closest["y_target"]],
    z=[closest["z_target"]],
    mode="markers",
    marker=dict(size=7),
    name="Target @ closest"
))

# Connecting line (minimum distance)
fig.add_trace(go.Scatter3d(
    x=[closest["x_hunter"], closest["x_target"]],
    y=[closest["y_hunter"], closest["y_target"]],
    z=[closest["z_hunter"], closest["z_target"]],
    mode="lines",
    name=f"Min distance = {closest['distance']:.2f} m",
    line=dict(dash="dash")
))

# Layout
fig.update_layout(
    title="Interactive 3D Drone Intercept – Closest Point Highlighted",
    scene=dict(
        xaxis_title="X (m)",
        yaxis_title="Y (m)",
        zaxis_title="Z (m)",
        aspectmode="data"
    ),
    legend=dict(itemsizing="constant")
)

fig.write_html("intercept_3d.html", auto_open=True)
#fig.show()

