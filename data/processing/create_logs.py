import cosysairsim as airsim
import pandas as pd
import time

client = airsim.MultirotorClient(ip='192.168.144.67')
hunter = "O4"
target = "target"

# draw position data

df = pd.DataFrame()
try:
    print('creating logs, press ^C')
    while True:
        new_df = pd.DataFrame({"x": [client.simGetObjectPose(hunter).position.x_val, client.simGetObjectPose(target).position.x_val],
                            "y": [client.simGetObjectPose(hunter).position.y_val, client.simGetObjectPose(target).position.y_val],
                            "z": [client.simGetObjectPose(hunter).position.z_val, client.simGetObjectPose(target).position.z_val],
                            "drone": ["hunter", "target"],
                            "timestamp": [time.time_ns()]*2})
        df = pd.concat([df, new_df], ignore_index=True)
except KeyboardInterrupt:
    now = time.localtime()
    filedate = time.strftime("%Y-%m-%d_%H_%M_%S", now)
    df.to_csv(f"logs/apn/{filedate}.csv")
    print(f"\nSaved to logs/apn/{filedate}.csv")
    
