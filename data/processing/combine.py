import pandas as pd
import numpy as np
from pathlib import Path

LOG_DIR = "logs/apn"          # change if needed
OUTPUT_FILE = "closest_apn.csv"

results = []

for i in range(1, 51):
    file_path = Path(LOG_DIR) / f"{i}.csv"
    if not file_path.exists():
        print(f"Skipping missing file: {i}.csv")
        continue

    df = pd.read_csv(file_path)
    df["z"] = -df["z"]

    hunter = df[df["drone"] == "hunter"].sort_values("timestamp")
    target = df[df["drone"] == "target"].sort_values("timestamp")

    merged = pd.merge(
        hunter,
        target,
        on="timestamp",
        suffixes=("_hunter", "_target")
    )

    if merged.empty:
        print(f"No aligned timestamps in {i}.csv")
        continue

    merged["distance"] = np.sqrt(
        (merged["x_hunter"] - merged["x_target"])**2 +
        (merged["y_hunter"] - merged["y_target"])**2 +
        (merged["z_hunter"] - merged["z_target"])**2
    )

    closest = merged.loc[merged["distance"].idxmin()]

    results.append({
        "flight": i,
        "min_distance_m": closest["distance"],
    })

summary_df = pd.DataFrame(results)
summary_df.to_csv(OUTPUT_FILE, index=False)

print(f"Saved results to {OUTPUT_FILE}")

