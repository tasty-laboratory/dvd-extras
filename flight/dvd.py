import cosysairsim as airsim
import numpy as np
import os
import tempfile
import pprint
import cv2
import time # Import time for delays

client = airsim.MultirotorClient(ip='192.168.144.67', port=41451)
pprint.pprint(client.listVehicles())

target_velocity = 2
circuit_length = 100
safe_altitude_z = -30
n_cycles = 10


vehicle_name = "target" # Ensure this matches your settings.json
client.enableApiControl(True, vehicle_name=vehicle_name)

pprint.pprint(client.getMultirotorState(vehicle_name=vehicle_name))

print(f"Arming {vehicle_name}...")
client.armDisarm(True, vehicle_name=vehicle_name)
print(f"{vehicle_name} armed.")

print(f"Taking off {vehicle_name}...")
client.takeoffAsync(vehicle_name=vehicle_name).join()
print(f"{vehicle_name} took off.")

# Ensure the drone is at a safe altitude before starting the circuit
# Z is NED, so negative Z is up. -30 means 30 meters above spawn point.
print(f"Moving {vehicle_name} to safe altitude {abs(safe_altitude_z)}m...")
client.moveToZAsync(safe_altitude_z, 2, vehicle_name=vehicle_name).join() # Move to -30m (30m up) at 2 m/s
# client.moveToPositionAsync(20, 0, safe_altitude_z, velocity=3, vehicle_name=vehicle_name).join()
print(f"{vehicle_name} at safe altitude.")
time.sleep(1) # Small pause to stabilize

# --- Define the Circuit Waypoints ---
# Waypoints are in NED coordinates (North-East-Down).
# X: North, Y: East, Z: Down (so negative Z is up)
# All waypoints should generally be at the same Z (altitude) for a horizontal circuit.
# Adjust these coordinates based on your Unreal environment's scale and desired circuit path.

# Let's define a square circuit at safe_altitude_z
# Start position (relative to initial spawn point)
# Current position after moveToZAsync should be close to (0, 0, safe_altitude_z)
# Assuming initial spawn is roughly (0,0,0) in world coordinates for simplicity
# and the -10m means 10m above this.
'''
cycle_waypoints = [
    airsim.Vector3r(circuit_length, 0, safe_altitude_z),
    airsim.Vector3r(0, 0, safe_altitude_z),
] * n_cycles
circuit_waypoints = cycle_waypoints
'''
# You can add more complex shapes, e.g., a circle by generating many points on a circle.
# Example for a circular path (more complex, but illustrates arbitrary circuit):
radius = 50 
num_points = 360
center_x, center_y = 0, 0 # Center of the circle relative to spawn
circular_waypoints = []
for i in range(num_points):
    angle = 2 * np.pi * i / num_points
    x = center_x + radius * np.cos(angle)
    y = center_y + radius * np.sin(angle)
    circular_waypoints.append(airsim.Vector3r(x, y, safe_altitude_z))
circuit_waypoints = circular_waypoints * n_cycles # Use this if you want a circular path

# --- Fly the Circuit ---
# moveOnPathAsync(path, velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead, vehicle_name)
# path: List of AirSim Vector3r objects (waypoints)
# velocity: Speed to travel along the path (m/s)
# timeout_sec: Maximum time to complete the path. Use a large number or 3e+38 for effectively infinite.
# drivetrain: AirSim.DrivetrainType.ForwardOnly (drone moves forward, yaw adjusts to path)
#             or AirSim.DrivetrainType.MaxDegreeOfFreedom (drone can move sideways, yaw is independent)
# yaw_mode: AirSim.YawMode(is_rate, yaw_or_rate_value).
#           is_rate=False, yaw_or_rate_value=0 means drone always points forward along the path.
#           is_rate=False, yaw_or_rate_value=90 means drone points at a fixed 90 degrees yaw.
#           is_rate=True, yaw_or_rate_value=10 means drone rotates at 10 deg/s.
# lookahead: Distance to look ahead when calculating the path.
# adaptive_lookahead: Whether to dynamically adjust lookahead.

print(f"Flying {vehicle_name} along the defined circuit...")
try:
    client.moveOnPathAsync(
        circuit_waypoints,
        velocity=target_velocity, # 3 m/s
        timeout_sec=909092923, # 1 hour max to complete circuit
        drivetrain=airsim.DrivetrainType.ForwardOnly, # Drone points in direction of travel
        yaw_mode=airsim.YawMode(False, 0), # No fixed yaw, let it follow path
        lookahead=5, # Look 5 meters ahead for path smoothing
        adaptive_lookahead=1, # Enable adaptive lookahead
        vehicle_name=vehicle_name
    ).join()
    print(f"{vehicle_name} completed the circuit.")
except Exception as e:
    print(f"An error occurred during circuit flight: {e}")

# --- Going Home ---
print(f"{vehicle_name} going home...")
client.goHomeAsync(vehicle_name=vehicle_name).join()

# --- Landing and Disarming ---
print(f"Hovering {vehicle_name} before landing...")
client.hoverAsync(vehicle_name=vehicle_name).join()
time.sleep(5) # Give it a moment to stabilize

preland=-5
print(f"Moving {vehicle_name} to safe altitude {abs(preland)}m...")
client.moveToZAsync(preland, 2, vehicle_name=vehicle_name).join() # Move to -30m (30m up) at 2 m/s
print(f"{vehicle_name} at safe altitude.")
time.sleep(1) # Small pause to stabilize

print(f"Landing {vehicle_name}...")
client.landAsync(vehicle_name=vehicle_name).join()
print(f"{vehicle_name} landed.")

print(f"Disarming {vehicle_name}...")
client.armDisarm(False, vehicle_name=vehicle_name)
print(f"{vehicle_name} disarmed.")

client.enableApiControl(False, vehicle_name=vehicle_name)
print("API control disabled.")

pprint.pprint(client.getMultirotorState(vehicle_name=vehicle_name))
