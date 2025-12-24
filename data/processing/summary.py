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

print(closest)
