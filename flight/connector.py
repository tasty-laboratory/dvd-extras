import asyncio
from mavsdk import System
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt
import time
import json
import math
import numpy as np
import argparse
import zmq
import signal
import sys
import nest_asyncio

nest_asyncio.apply()

# Constants
DIST_PER_DEG_LAT = 110574.9013
DIST_PER_DEG_LONG = 111288.3423

# Global drone position
latitude = 0.0
longitude = 0.0
altitude = 0.0

# Global shutdown event
shutdown_event = None



# Signal handler
def shutdown_handler(signum, frame):
    print("\nShutdown signal received. Cleaning up...")
    shutdown_event.set()

# Register signal handlers (for both SIGINT and SIGTERM)
signal.signal(signal.SIGINT, shutdown_handler)
signal.signal(signal.SIGTERM, shutdown_handler)

# ===== Utility Functions =====

def vector_magnitude(vector):
    # Calculate the sum of squares of each component
    sum_of_squares = sum(comp ** 2 for comp in vector)
    
    # Calculate the magnitude by taking the square root of the sum
    magnitude = math.sqrt(sum_of_squares)
    
    return magnitude


def normalize_eagle_line(A, B):
    """
    Given two GPS coordinates A and B in [latitude, longitude] format,
    this function calculates a new point C that lies on the same line as A→B,
    but at a fixed distance (150 metres) from A towards B.

    It does so by:
    1. Converting latitude/longitude into metre-based cartesian coordinates.
    2. Finding the direction vector from A to B.
    3. Normalising it to 150 metres.
    4. Adding it to point A to get point C.
    5. Converting C back to latitude/longitude for return.
    """

    A = np.array(A)
    B = np.array(B)

    A[0] = A[0] * DIST_PER_DEG_LAT
    A[1] = A[1] * DIST_PER_DEG_LONG

    B[0] = B[0] * DIST_PER_DEG_LAT
    B[1] = B[1] * DIST_PER_DEG_LONG

    AB = B - A
    AB_magnitude = vector_magnitude(AB)

    # Normalize vector AB and scale it to 200 metres
    AB = AB / AB_magnitude * 500
    
    # Compute new point C by moving 150m from A towards B
    C = A + AB

    C[0] = C[0] / DIST_PER_DEG_LAT
    C[1] = C[1] / DIST_PER_DEG_LONG

    return C

def calculate_bearing(lat1, long1, lat2, long2):
    lat1 = math.radians(lat1)
    long1 = math.radians(long1)
    lat2 = math.radians(lat2)
    long2 = math.radians(long2)

    dLong = long2 - long1

    y = math.sin(dLong) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLong)

    bearing = math.degrees(math.atan2(y, x))

    # normalize to 0° to 360°
    bearing = (bearing + 360) % 360

    return bearing


# ===== Drone Telemetry =====

async def store_drone_position(drone):
    global latitude, longitude, altitude
    async for position in drone.telemetry.position():
        # info = {}
        latitude = position.latitude_deg
        longitude =  position.longitude_deg
        altitude = position.relative_altitude_m
        # print(latitude, longitude, altitude)

# ====== AirSim Drone Telemetry======

async def store_airsim_drone_position(drone, vehicle_name):
    global latitude, longitude, altitude
    while not shutdown_event.is_set(): # Check for shutdown signal
        try:
            # Run the synchronous (blocking) AirSim call in a separate thread
            # state = await asyncio.to_thread(drone.getMultirotorState, vehicle_name=vehicle_name)
            state = drone.getMultirotorState(vehicle_name=vehicle_name)
            gps = state.gps_location
            
            latitude = gps.latitude
            longitude = gps.longitude
            altitude = gps.altitude

            print(f"[Telemetry] {latitude}, {longitude}, {altitude}")
            await asyncio.sleep(0.5)
        
        except Exception as e:
            print(f"Error in AirSim telemetry loop: {e}")
            # If the connection is lost, you might want to break or try to reconnect
            if "Connection refused" in str(e) or "Connection lost" in str(e):
                print("Connection to AirSim lost. Stopping telemetry task.")
                break
            await asyncio.sleep(1) # Wait a bit before retrying on other errors
# ===== Publish Messages =====

async def publish_default(time_interval):
    """ Publishes camera metadata and eagle line position """
    tower_lat, tower_lon, tower_alt = 1.2808889, 103.8166889, 0
    stream_url = "rtsp://localhost:8554/annotated"

    while True:
        target = normalize_eagle_line(
            [tower_lat, tower_lon, tower_alt],
            [latitude, longitude, altitude]
        )

        yaw = calculate_bearing(tower_lat, tower_lon, latitude, longitude)

        metadata = {
            "deviceID": "T1",
            "latitude": tower_lat,
            "longitude": tower_lon,
            "altitude": tower_alt,
            "pitch": 45.0,
            "roll": 0.0,
            "yaw": yaw + 180,
            "fov": 58.1,
            "streamURL": stream_url,
            "detect": True,
            "range": 50,
            "zoom": 1.0,
            "northYaw": 0.0,
        }

        eagle_line = {
            "deviceID": "T1",
            "source": "zt30",
            "latitude1": tower_lat,
            "longitude1": tower_lon,
            "altitude1": tower_alt,
            "latitude2": target[0],
            "longitude2": target[1],
            "altitude2": target[2],
        }

        publish.single("METADATA", json.dumps(metadata), hostname="192.168.144.50")
        publish.single("EAGLELINE", json.dumps(eagle_line), hostname="192.168.144.50")

        # print("[Default] Published METADATA and EAGLELINE")
        await asyncio.sleep(time_interval)
        
async def publish_radar(time_interval):
    """ Publish radar-format message to 'track/global/1000' """
    track_id = "1000"
    topic = f"track/global/{track_id}"


    airsim_offset = 50

    while True:
        radar = {
            "occurredOn": int(time.time() * 1000),
            "deletion": False,
            "detail": {
                "geoPosition": {
                    "latitude": latitude,
                    "longitude": longitude,
                    "height": altitude - airsim_offset,
                    "heightType": "AGL"
                }
            }
        }


        publish.single(topic, json.dumps(radar), hostname="192.168.144.50")
        print(f"[Radar] Published to {topic}: {radar}")
        await asyncio.sleep(time_interval)

# ===== ZMQ Publisher for MAVIC =====

async def store_drone_position_from_zmq():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://192.168.144.50:5560") 
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    print("Listening for ZMQ drone position updates...")

    eaglepoint = {
        "occurredOn": int(time.time() * 1000),
        "deletion": False,
        "detail": {
            "geoPosition": {
                "latitude": latitude,
                "longitude": longitude,
                "height": altitude,
                "heightType": "AMSL"
            }
        }
    }

    while True:
        try: 
            # Get the reply.
            parts = await asyncio.to_thread(socket.recv_multipart)
            _, data = parts
            decoded = data.decode('utf-8')
            message = json.loads(decoded)[0]

            if isinstance(message, dict):
                # Pause for 0.5s to simulate 0.5 hertz
                await asyncio.sleep(0.5)
                eaglepoint["detail"]["geoPosition"]["latitude"] = message['latitude'] / 1e7
                eaglepoint["detail"]["geoPosition"]["longitude"] = message['longitude'] / 1e7
                eaglepoint["detail"]["geoPosition"]["height"] = (message['altitude'] / 2) - 1000
                print(eaglepoint)
                publish.single("track/global/MAVIC", json.dumps(eaglepoint), hostname="localhost")
    
        except Exception as e:
            print(f'Error: {e}')
            await asyncio.sleep(0.5)

async def main():
    # Flag implemented to switch between gazebo and live testing drone broadcasting GTruth [MAVIC]
    # By default, gazebo drone is selected, to select the live drone, run: "python3 connector.py --source live"
    global shutdown_event
    shutdown_event = asyncio.Event()
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', choices=['gazebo', 'airsim', 'live'], default='airsim')
    args = parser.parse_args()
    tasks = []

    if args.source == 'gazebo':
        print("🟢 Using GAZEBO drone via MAVSDK...")
        # Connections to MAVSDK
        # drone = System(mavsdk_server_address='10.168.10.85', port=50061)
        drone = System(mavsdk_server_address='localhost', port=50061)
        await drone.connect(system_address="udp://:14541")

        async for state in drone.core.connection_state():
            if state.is_connected:
                print("-- Connected to GAZEBO drone")
                break

        # Start MAVSDK telemetry
        tasks.append(asyncio.create_task(store_drone_position(drone)))

        # Start radar + metadata tasks
        time_interval = 0.5
        tasks.append(asyncio.create_task(publish_radar(time_interval)))
        tasks.append(asyncio.create_task(publish_default(time_interval)))
    elif args.source == 'airsim':
        import cosysairsim as airsim

        print("🟢 Using AirSim drone via AirSim Simple-Flight API")
        # AirSim
        drone = airsim.MultirotorClient(ip='192.168.144.67')

        vehicle_name = "target" # Ensure this matches your settings.json
        
        # Start AirSim telemetry
        tasks.append(asyncio.create_task(store_airsim_drone_position(drone, vehicle_name)))
        # Start radar + metadata tasks
        time_interval = 0.5
        tasks.append(asyncio.create_task(publish_radar(time_interval)))
        # tasks.append(asyncio.create_task(publish_default(time_interval)))
        pass
    else:
        print("🔵 Using LIVE drone via ZMQ...")
        tasks.append(asyncio.create_task(store_drone_position_from_zmq()))

    await shutdown_event.wait()
    print("*** Shutdown event detected, cancelling tasks... ***")

    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)

    print("*** Shutdown complete. ***")

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
