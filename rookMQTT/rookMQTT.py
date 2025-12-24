# Uses FPS optimizations from VINO as well as alternate method of streaming video
import asyncio
import math
import queue
import re
from time import time

from async_timeout import timeout
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, VelocityNedYaw)
import numpy as np

from . import pid

from cfg import DroneState as drone_state
from cfg import Os as os
from cfg import OS, MAVSDK_SERVER_IP, INPUT_FRAME_WIDTH, INPUT_FRAME_HEIGHT

# ===== Parameters =====
# Autonomous Decision Parameters

DETECTTIMEOUT = 20 # how many loops of no detection before switching from Follow/DASH to Search


# Eagleline Parameters
STARTPOINTDISTANCEFROMTOWER = 20
LEADDISTANCETOAIMFORALONGEAGLELINE = 20
EAGLELINESPEED = 5

# Eaglepoint Parameters (Essentially a PD controller)
EP_ALT_OFFSET = 2  # How many meters below the target do we want the Hunter to perform guidance
K = 1 # Lead Coefficient (0 means no lead, 1 gives a lead equivalent to how far the target has moved since the previous loop)
HORI_GAIN = 0.1 # Proportional Gain
HORI_MIN_SPEED = 0.0 # 1.0 # set to 0.0 to prevent crashes, increase otherwise
HORI_MAX_SPEED = 10.0
VERT_GAIN = 0.15 # Proportional Gain
VERT_MIN_SPEED = 0.0 # 1.0 # set to 0.0 to prevent crashes, increase otherwise
VERT_MAX_SPEED = 2.0
EAGLEPOINT_DASH_PROPORTIONAL_GAIN = 4
EAGLEPOINT_DASH_VELOCITY =20 

#EAGLEGUARD PARAMETERS ()
EPG_KP_HORIZONTAL = 0.35
EPG_KP_VERTICAL = 0.08
EPG_HORIZONTAL_DEADBAND = 2  # meters
EPG_VERTICAL_DEADBAND = 1    # meters  
EPG_SLOWDOWN_RADIUS = 25.0       # meters (Horizontal)
EPG_SLOWDOWN_HEIGHT = 2.5        # meters (Vertical)
EPG_MAX_HORIZONTAL_SPEED = 10.0  # m/s
EPG_MAX_VERTICAL_SPEED = 2.0     # m/s
EPG_ALTITUDE_OFFSET = 10.0        # meters below the target
EAGLEGUARD_HORIZ_DASH_PROPORTIONAL_GAIN = 3
EAGLEGUARD_VERT_DASH_PROPORTIONAL_GAIN = 4
EPG_SAFETY_ALTITUDE = 10.0  # Minimum altitude when guarding a point

FRAMESIZE = (INPUT_FRAME_WIDTH, INPUT_FRAME_HEIGHT)

# PID Parameters
YOFFSETDASH = 0
ZOFFSETDASH = 0
YOFFSETFOLLOW = 4
ZOFFSETFOLLOW = 10 
KPYAW = 20.0
KPTRANS = 0.28
BETAYAW = 1
BETATRANS = 1
VZMAX = 14 #Forward
VYUPMAX = 8 #Vertical Up
VYDOWNMAX = 3 #Vertical Down
VXMAX = 14 #Horizontal
VYAWMAX = 45 #Yaw
SEARCH_YAW = 0 # Yaw rate in degrees per second

# Singapore
# DIST_PER_DEG_LAT = 110574.9013
# DIST_PER_DEG_LONG = 111288.3423 # TODO: generalise value for diff locs

# Shoalwater Bay Training Area
DIST_PER_DEG_LAT = 110737
DIST_PER_DEG_LONG = 102913 

previous_omega = np.zeros(3)
pid_error = np.zeros((3, 4))
pid_signal = np.zeros((3, 4))
log_width = np.zeros((2,1))
filtered_log_width_rate = np.zeros((2,1))
distance_history = np.zeros((2,1))
filtered_distance_history = np.zeros((2,1))
px_history_x = np.zeros((2,1))
px_history_y = np.zeros((2,1))
filtered_px_rate_history_x = np.zeros((2,1))
filtered_px_rate_history_y = np.zeros((2,1))
prev_yaw = 0
info = None

class ReplayBuffer():
    def __init__(self, capacity=2):
        self._capacity = capacity 
        self.memory = queue.Queue(maxsize=self.capacity)
        self.position = 0

    def __len__(self):
        return self.memory.qsize()

    @property
    def capacity(self):
        return self._capacity

    @capacity.setter
    def capacity(self, new_capacity):
        if new_capacity >= 0:
            self._capacity = new_capacity
        else:
            print("Value cannot be negative.")

    def add(self, coords):
        # If memory is full, remove the earliest coordinates
        if self.memory.full():
            self.memory.get()
        self.memory.put(coords)

    def peek(self, index):
        return self.memory.queue[index]
    
    def get_all_coords(self):
        coords = []
        for i in range(self.memory.qsize()):
            coords.append(self.memory.queue[i])
        return np.array(coords)
    
class EPGState:
    def __init__(self):
        self.guard_position = None
        self.target_yaw = None
        self.target_altitude = None
        self.is_active = False
        self.height_type = 0 
        
    def reset(self):
        self.guard_position = None
        self.target_yaw = None
        self.target_altitude = None
        self.is_active = False
        self.height_type = 0 
        
    def set_guard_position(self, position, yaw=None):
        self.guard_position = position
        if yaw is not None:
            self.target_yaw = yaw
        self.is_active = True
        
    def set_target_tracking(self, drone_point, eagle_point):
        if eagle_point:
            self.target_yaw = calculate_bearing(
                drone_point[0], drone_point[1], 
                eagle_point[0], eagle_point[1]
            )
            if len(eagle_point) > 2 and eagle_point[2] != 0:
                self.target_altitude = float(eagle_point[2])

async def resetPIDFollow(currenterror):
    global pid_error, pid_signal
    pid_error[0] = currenterror
    pid_error[1] = currenterror
    pid_error[2] = currenterror
    pid_signal[0, 3] = min(max(BETATRANS*KPTRANS*pid_error[0, 3],-VZMAX),VZMAX) # Forward Speed in m/s, +ve is forward
    pid_signal[0, 2] = min(max(BETATRANS*KPTRANS*pid_error[0, 2],-VYUPMAX),VYDOWNMAX) # Down Speed in m/s, +ve is downward
    pid_signal[0, 1] = min(max(BETATRANS*KPTRANS*pid_error[0, 1],-VXMAX),VXMAX) # Right Speed in m/s, +ve is right
    pid_signal[0, 0] = min(max(BETAYAW*KPYAW*pid_error[0, 0],-VYAWMAX),VYAWMAX) # in deg/s, +ve is clockwise
    pid_signal[1] = pid_signal[0]
    pid_signal[2] = pid_signal[0]

async def resetPIDDash(currenterror):
    global pid_error, pid_signal
    pid_error[0] = currenterror
    pid_error[1] = currenterror
    pid_error[2] = currenterror
    pid_signal[0, 3] = min(max(pid_signal[0, 3]+BETATRANS*KPTRANS*pid_error[0, 3],-VZMAX),VZMAX) # Forward Speed in m/s, +ve is forward
    pid_signal[0, 2] = min(max(pid_signal[0, 2]+BETATRANS*KPTRANS*pid_error[0, 2],-VYUPMAX),VYDOWNMAX) # Down Speed in m/s, +ve is downward
    pid_signal[0, 1] = min(max(BETATRANS*KPTRANS*pid_error[0, 1],-VXMAX),VXMAX) # Right Speed in m/s, +ve is right
    pid_signal[0, 0] = min(max(BETAYAW*KPYAW*pid_error[0, 0],-VYAWMAX),VYAWMAX) # in deg/s, +ve is clockwise
    pid_signal[1] = pid_signal[0]
    pid_signal[2] = pid_signal[0]

async def reset_log_width():
    global log_width, filtered_log_width_rate
    log_width = np.zeros((2,1))
    filtered_log_width_rate = np.zeros((2,1))

async def reset_log_distance():
    global distance_history, filtered_distance_history
    distance_history = np.zeros((2,1))
    filtered_distance_history = np.zeros((2,1))

async def reset_px_history():
    global filtered_px_rate_history_x, px_history_x
    global filtered_px_rate_history_y, px_history_y
    px_history_x = np.zeros((2,1))
    filtered_px_rate_history_x = np.zeros((2,1))
    px_history_y = np.zeros((2,1))
    filtered_px_rate_history_y = np.zeros((2,1))

async def reset_prev_yaw():
    global prev_yaw
    prev_yaw = 0


async def getPixelCoordsFromBBox(bbox):
    if bbox is None:
        return None
    return [bbox[0] + (bbox[2] - FRAMESIZE[0])//2, bbox[1] + (bbox[3] - FRAMESIZE[1])//2]

async def init():
    print("Initializing...")
    if OS == os.UBUNTU:
        drone = System(mavsdk_server_address='localhost', port=50060)
        await drone.connect(system_address="udp://:14542")
        # drone = System(mavsdk_server_address='192.168.0.123', port=50060)
        # await drone.connect(system_address="udp://:14540")
    elif OS == os.WINDOWS:
        drone = System(mavsdk_server_address='localhost', port=50051)
        await drone.connect(system_address="serial:///COM3:921600")
    elif OS == os.BLUE:
        drone = System()
        await drone.connect(system_address="udp://:14445")
    elif OS == os.GAZEBO:
        drone = System(mavsdk_server_address=MAVSDK_SERVER_IP, port=50060)
        await drone.connect(system_address="udp://:14542")
    elif OS == os.AIRSIM:
        drone = System(mavsdk_server_address=MAVSDK_SERVER_IP, port=50060)
        await drone.connect(system_address="udp://:14542")

    replayBuffer = ReplayBuffer()
    
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            return (drone, replayBuffer)

def decode_coordinates(coordinates_str, msgtype="eagle_line"):
    # Use regular expression to extract the coordinates
    matches = re.findall(r'\[(.*?)\]', coordinates_str)

    # Initialize a dictionary to store the coordinates
    coordinates_dict = {}

    if msgtype == "eagle_line":
        # Iterate through the matches and split the string into individual coordinates
        for i, match in enumerate(matches, 1):
            coords = [float(coord) for coord in match.split(',')]
            if i == 1:
                coordinates_dict["Point1"] = coords
            elif i == 2:
                coordinates_dict["Point2"] = coords
            else:
                coordinates_dict[f"Coordinate {i}"] = coords
    elif msgtype == "eagle_point":
        # Iterate through the matches and split the string into individual coordinates
        for i, match in enumerate(matches, 1):
            if i == 1:
                coords = [float(coord) for coord in match.split(',')]
                coordinates_dict["Point"] = coords
            elif i == 2:
                coordinates_dict["Type"] = match
            elif i == 3:
                coordinates_dict["HType"] = int(match)
            else:
                coordinates_dict[f"Coordinate {i}"] = coords
    elif msgtype == "eagle_guard":
        for i, match in enumerate(matches, 1):
            if i == 1:
                coords = [float(coord) for coord in match.split(',')]
                coordinates_dict["Guard_Point"] = coords
            elif i == 2:
                coordinates_dict["Yaw"] = float(match)
            else:
                coordinates_dict[f"Coordinate {i}"] = coords
    return coordinates_dict

def vector_magnitude(vector):
    # Calculate the sum of squares of each component
    sum_of_squares = sum(comp ** 2 for comp in vector)
    
    # Calculate the magnitude by taking the square root of the sum
    magnitude = math.sqrt(sum_of_squares)
    
    return magnitude

# ===== For Eagle Line (EL) =====
# Function to calculate the displacement between point C and where hunter should go on line segment AB
# C is the drone's current position, B is the end of the eagle line, A is the tower
# D is the point on the line segment AB that is closest to C, E is the displacement between where hunter is and where it should be at
def get_displacement_to_line_segment(A, B, C, is_reached_eagleline):

    MINIMUM_DISTANCE = 5 # Distance to move at each interation towards the target

    # Convert points to numpy arrays for easier computation
    A = np.array(A)
    B = np.array(B)
    C = np.array(C)
    E = np.array([0, 0, 0])

    yaw_deg = calculate_bearing(C[0], C[1], B[0], B[1])

    # Change from lat, long to meters
    A = geo_unit_conversion(A, deg2meters=True)
    B = geo_unit_conversion(B, deg2meters=True)
    C = geo_unit_conversion(C, deg2meters=True)

    # Vector representing the line segment AB
    AB = B - A
    ABNorm = AB/vector_magnitude(AB)
    # Vector representing the line segment AC
    AC = C - A

    # Vector from current_drone to target_drone
    BC = C - B
    # Allow the drone to always face the target drone
    # yaw_deg = get_yaw_deg([BC[0], BC[1]])

    # Projection of AC onto AB
    AC_projection = np.dot(AC, ABNorm) * ABNorm

    # Coordinates of point D
    D = A + AC_projection

    # Perpendicular path to the eagle line
    CD_projection = D - C
    CD_magnitude = vector_magnitude(CD_projection)
    
    # print(CD_magnitude)
    # print(is_reached_eagleline)
    if not is_reached_eagleline:
        first_point_target = A + STARTPOINTDISTANCEFROMTOWER * ABNorm
        E = first_point_target - C
        distance_to_first_point = vector_magnitude(E)
        return E, yaw_deg, (distance_to_first_point < MINIMUM_DISTANCE)
    
    # Following the Eagle Line upwards to the end since it is close enough to the eagle line
    if(CD_magnitude < MINIMUM_DISTANCE):
        CB_magnitude = vector_magnitude(BC)
        
        # Reached the end of the line
        if (CB_magnitude < MINIMUM_DISTANCE):
            return None, yaw_deg, True
        intermediate_point_target = D + LEADDISTANCETOAIMFORALONGEAGLELINE * ABNorm
        E = intermediate_point_target - C
        # print("moving along the eagle line")

    # Moving perpendicular towards the eagle line
    else:
        # if (CD_magnitude > MINIMUM_DISTANCE * 10):
        #     return E, yaw_deg, False
        # print("moving perpendicular to the line, ", CD_magnitude)
        E = CD_projection
    
    return E, yaw_deg, True

# ===== For Eagle Point (EP) =====
# Function to calculate the displacement between hunter and prey drone and where hunter should go towards the drone
def get_displacement_to_point(curr_hunter_coords, curr_prey_pos, replay_buffer):
    curr_prey_coords = curr_prey_pos[:3]

    replay_buffer.add(curr_prey_pos)
    prev_prey_coords = replay_buffer.peek(0)[:3]
    
    # Convert points to numpy arrays for easier computation
    A = np.array(curr_hunter_coords)
    B = np.array(prev_prey_coords)
    C = np.array(curr_prey_coords)
    D = np.array([0, 0, 0]) # target coordinate
    E = np.array([0, 0, 0]) # direction vector

    # Change from lat, long to meters
    A = geo_unit_conversion(A, deg2meters=True)
    B = geo_unit_conversion(B, deg2meters=True)
    C = geo_unit_conversion(C, deg2meters=True)
    
    # If there are sufficient coordinates (2), calculate extrapolation else instruct the drone to move towards the only coordinate detected
    D = calculate_lead(B, C, K)
    
    # Calculate direction vector
    E = D - A

    # Convert back to degrees to calculate bearing
    A = geo_unit_conversion(A, deg2meters=False)
    D = geo_unit_conversion(D, deg2meters=False)

    yaw_deg = calculate_bearing(A[0], A[1], D[0], D[1])

    return E, yaw_deg, replay_buffer

# Estimate lead direction to move based on the prey drone's previous positions
def calculate_lead(prev, curr, K):
    direction = curr - prev
    lead_vector = direction * K
    target_pos = curr + lead_vector
    return target_pos

# Calculate gain based on distance
def calculate_speed_gain(target_point):
    horizontal_target = target_point[:2] # consist of North and East
    vertical_target = target_point[2] # consist of Down

    horizontal_dist = np.linalg.norm(horizontal_target)
    vertical_dist = abs(vertical_target)
    
    horizontal_speed = max(HORI_MIN_SPEED, min(HORI_MAX_SPEED, horizontal_dist * HORI_GAIN))
    vertical_speed = max(VERT_MIN_SPEED, min(VERT_MAX_SPEED, vertical_dist * VERT_GAIN))

    # Scale direction vector
    if horizontal_dist != 0: 
        horizontal_direction = horizontal_target / horizontal_dist
        NSpeed = horizontal_direction[0] * horizontal_speed
        ESpeed = horizontal_direction[1] * horizontal_speed
    else:
        NSpeed = 0.0
        ESpeed = 0.0

    if vertical_dist != 0:
        USpeed = -vertical_speed * np.sign(vertical_target) # negative because down is positive
    else:
        USpeed = 0.0

    print(f"Horizontal dist: {horizontal_dist:.2f}m, Vertical dist: {vertical_dist:.2f}m")
    print(f"Speeds - N: {NSpeed:.2f}, E: {ESpeed:.2f}, U: {USpeed:.2f}")
    return NSpeed, ESpeed, USpeed

# ===== For Eagle Point Guard (EPG) =====
# Function to calculate the displacement between hunter and prey drone and where hunter should go towards the drone
def get_displacement_to_guard(hunter_point, target_coords):
    A = np.array(hunter_point)

    if target_coords[2] is None:
        target_coords[2] = hunter_point[2]

    B = np.array(target_coords)

    # Convert lat, lon, alt to meters for both points
    A = geo_unit_conversion(A, deg2meters=True)
    B = geo_unit_conversion(B, deg2meters=True)

    displacement = B - A

    yaw_deg = calculate_bearing(hunter_point[0], hunter_point[1], target_coords[0], target_coords[1])

    return displacement, yaw_deg


epg_state = EPGState()

# Process the 2 EPG messages coming in
def process_epg_message(msg, drone_point):
    """Process EPG message and update state."""
    if not msg or not drone_point:
        return False
        
    subtype = msg.get('subtype')
    height_type = msg.get('HType', 0)
    
    if subtype == 'guard_position':
        guard_point = msg.get('Guard_Point')
        if guard_point:
            print("Received new guard position...")
            yaw = msg.get('Yaw')
            epg_state.set_guard_position(guard_point, yaw)
            epg_state.target_altitude = None
            return True
            
    elif subtype == 'yaw_and_altitude':
        eagle_point = msg.get('Point')
        if eagle_point:
            epg_state.height_type = height_type 
            epg_state.set_target_tracking(drone_point, eagle_point)
            return True
            
    return False

# Calculate the velocity based on EPG provided
def calculate_epg_movement(drone_point):
    """Calculate EPG movement speeds with simple P control. Returns (NSpeed, ESpeed, USpeed, Yaw) or None."""
    if not epg_state.is_active or not epg_state.guard_position or not drone_point:
        return None

    target_coords = list(epg_state.guard_position)
    if epg_state.target_altitude is not None:
        target_coords[2] = epg_state.target_altitude - EPG_ALTITUDE_OFFSET
    elif len(target_coords) <= 2 or target_coords[2] in (None, 0.0):
        target_coords.append(drone_point[2])  # Use current altitude

    # Enforce minimum altitude if height type is AGL
    if (target_coords[2] < EPG_SAFETY_ALTITUDE) and (epg_state.height_type == 0):
        target_coords[2] = EPG_SAFETY_ALTITUDE
        
    displacement, yaw_from_guard = get_displacement_to_guard(drone_point, target_coords)
    if displacement is None:
        return None
        
    final_yaw = epg_state.target_yaw if epg_state.target_yaw is not None else yaw_from_guard
    
    horizontal_vector = np.array([displacement[0], displacement[1]])
    horizontal_distance = np.linalg.norm(horizontal_vector)
    vertical_distance = abs(displacement[2])
    
    ## HORIZONTAL CONTROL ##
    if horizontal_distance <= EPG_HORIZONTAL_DEADBAND:
        #within deadband, come to a stop
        n_speed, e_speed = 0.0, 0.0
    elif horizontal_distance <= EPG_SLOWDOWN_RADIUS:
        #within slowdown, apply horizontal p control
        n_speed = np.clip(displacement[0] * EPG_KP_HORIZONTAL, 
                         -EPG_MAX_HORIZONTAL_SPEED, EPG_MAX_HORIZONTAL_SPEED)
        e_speed = np.clip(displacement[1] * EPG_KP_HORIZONTAL, 
                         -EPG_MAX_HORIZONTAL_SPEED, EPG_MAX_HORIZONTAL_SPEED)
    else:
        #outside deadband - max speed
        unit_horizontal = horizontal_vector / horizontal_distance
        n_speed = unit_horizontal[0] * EPG_MAX_HORIZONTAL_SPEED
        e_speed = unit_horizontal[1] * EPG_MAX_HORIZONTAL_SPEED
    
    ##VERTICAL CONTROL ##
    if vertical_distance <= EPG_VERTICAL_DEADBAND:
        #within deadband, come to a stop
        u_speed = 0.0
    elif vertical_distance <= EPG_SLOWDOWN_HEIGHT:
        u_speed = np.clip(displacement[2] * -EPG_KP_VERTICAL, -EPG_MAX_VERTICAL_SPEED, EPG_MAX_VERTICAL_SPEED)
    else:
        #outside deadband - max speed
        u_speed = -EPG_MAX_VERTICAL_SPEED if displacement[2] > 0 else EPG_MAX_VERTICAL_SPEED
        
    return n_speed, e_speed, u_speed, final_yaw

# ===== General Functions =====
def geo_unit_conversion(coords, deg2meters=True):
    coords_converted = coords.copy()
    if deg2meters:
        coords_converted[0] = coords[0] * DIST_PER_DEG_LAT
        coords_converted[1] = coords[1] * DIST_PER_DEG_LONG
    else:
        coords_converted[0] = coords[0] / DIST_PER_DEG_LAT
        coords_converted[1] = coords[1] / DIST_PER_DEG_LONG
    return coords_converted

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

def vector_magnitude(vector):
    # Calculate the sum of squares of each component
    sum_of_squares = sum(comp ** 2 for comp in vector)
    
    # Calculate the magnitude by taking the square root of the sum
    magnitude = math.sqrt(sum_of_squares)
    
    return magnitude

async def store_drone_position(drone):
    global info
    async for position in drone.telemetry.position():
        info = {}
        info['lat'] = position.latitude_deg
        info['long'] = position.longitude_deg
        info['altAGL'] = position.relative_altitude_m # above ground level
        info['altAMSL'] = position.absolute_altitude_m # above mean sea level

def get_state_name(state_value):
    for state in drone_state:
        if state.value == state_value:
            return state.name
    return f"UNKNOWN({state_value})"

async def main(topics_in, topics_out):
    # topics_in: msg (MQTT), bbox (CV)
    # topics_out: mode (MQTT), pid_info (drawer/RC3)
    MQTT_recv_topic = topics_in[1]
    bbox_topic = topics_in[0]
    MQTT_send_topics = topics_out[0]
    RC3_send_topics = topics_out[1]

    # Set by default, drone will toggle to auto follow mode.
    detection_enabled = True  

    closing_speed = None
    corrected_speeds = None
    speeds = None
    is_offboard = False
    current_state = drone_state.SEARCH.value
    (drone, replay_buffer) = await init()
    px_error = (0,0)
    last_target_range = 0
    previous_velocity = None
    NSpeed = 0 
    ESpeed = 0 
    USpeed = 0
    
    while True:
        #messaging In
        msg = MQTT_recv_topic.pull()
        asyncio.ensure_future(store_drone_position(drone))
        # drone_point = None # Hardcoded point
        drone_point = eagle_point = None # Hardcoded point
        target_point = tower_point = end_eagle_line_point = None
        yaw_deg = 0
        is_reach_eagle_line = False
        is_start_eagle_line = False
        is_start_eagle_point = False
        is_start_eagle_point_guard = False

        while True:
            ptime = time()
        #messaging In
            await asyncio.sleep(0.01) # Let the position thread to run
            msg = MQTT_recv_topic.pull()
        #Inputs
            bbox = bbox_topic.pull()
            errors = pid.calculate_errors(bbox,0)
            if errors is None:
                offsetedErrors = None
                closing_speed = None
            else:
                offsetedErrors = np.array(errors[:4])
                last_target_range = errors[3]
                px_error = (errors[4],errors[5])
                current_log_width = errors[6]
            target_pixel_coords = await getPixelCoordsFromBBox(bbox)
        #Autonomous Decision
            if current_state == drone_state.DASHING.value or current_state == drone_state.GUIDANCEDASH.value:
                Zo = ZOFFSETDASH
                Yo = YOFFSETDASH
            else:
                Zo = ZOFFSETFOLLOW
                Yo = YOFFSETFOLLOW
            if offsetedErrors is not None: # detection box
                offsetedErrors[2] += Yo
                offsetedErrors[3] -= Zo
                no_dect_counter = 0
                # Add here to followEL if want it to be autonomous
                if (current_state in [drone_state.EPG.value]) and detection_enabled:
                    await resetPIDFollow(offsetedErrors)
                    current_state = drone_state.GUARDDASH.value
                    print("Drone detected & switched to following.")
                # elif (current_state in [drone_state.SEARCH.value, drone_state.EL.value, drone_state.EP.value]) and detection_enabled:
                ############# Commented out to disable following mode #################
                #     await resetPIDFollow(offsetedErrors)
                #     current_state = drone_state.FOLLOWING.value
                #     print("Drone detected & switched to following.")
            elif (current_state == drone_state.FOLLOWING.value) or (current_state == drone_state.DASHING.value) or (current_state == drone_state.GUARDDASH.value) or (current_state == drone_state.GUIDANCEDASH.value):
                no_dect_counter += 1
                if no_dect_counter >= DETECTTIMEOUT or not detection_enabled:
                    is_reach_eagle_line = False
                    is_start_eagle_line = False
                    is_start_eagle_point = False
                    is_start_eagle_point_guard = False
                    target_point = tower_point = end_eagle_line_point = None
                    previous_velocity = None
                    await reset_log_width()
                    await reset_log_distance()
                    await reset_px_history()
                    await reset_prev_yaw()
                    if current_state == drone_state.GUARDDASH.value:
                        current_state = drone_state.EPG.value
                        is_start_eagle_point_guard = True
                    else:
                        current_state = drone_state.SEARCH.value
                        is_start_eagle_point_guard = False

        #Manual Decision
            if msg is not None:
                curr_drone_state = int(msg.get('command'))
                print(f"Received MQTT command: {curr_drone_state} ({get_state_name(curr_drone_state)})")
                if curr_drone_state == drone_state.LAUNCH.value:
                    try:
                        droneIsHealthy = False
                        async for health in drone.telemetry.health():
                            if health.is_global_position_ok and health.is_home_position_ok:
                                print("Drone has a valid position estimate.")
                                droneIsHealthy = True
                            else:
                                print("Drone needs a valid position before launch.")
                                droneIsHealthy = False
                            break
                        if droneIsHealthy:
                            print("healthy")
                            await drone.action.arm()
                            await drone.action.takeoff()
                    except ActionError as error:
                        print(f"Launch failed: {error._result.result}")

                elif curr_drone_state == drone_state.LAND.value:
                    try:
                        droneIsHealthy = False
                        async for health in drone.telemetry.health():
                            if health.is_global_position_ok and health.is_home_position_ok:
                                print("Drone has a valid position estimate.")
                                droneIsHealthy = True
                            else:
                                print("Drone needs a valid position before landing.")
                                droneIsHealthy = False
                            break
                        if droneIsHealthy:
                            await drone.action.return_to_launch()
                    except ActionError as error:
                        print(f"Land failed: {error._result.result}")

                elif curr_drone_state == drone_state.OFFBOARD.value:
                    try:
                        print("-- Setting initial setpoint")
                        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                    except OffboardError as error:
                        print(f"Setting Setpoint failed with error code: \
                                {error._result.result}")
                    try:
                        print("-- Starting offboard")
                        await drone.offboard.start()
                    except OffboardError as error:
                        print(f"Starting offboard mode failed with error code: \
                                {error._result.result}")
                    if offsetedErrors is not None:
                        await resetPIDFollow(offsetedErrors)
                    is_offboard = True

                elif curr_drone_state == drone_state.ONBOARD.value:
                    is_offboard = False
                    target_point = tower_point = end_eagle_line_point = None
                    try:
                        await drone.offboard.stop()
                    except OffboardError as error:
                        print(f"Onboard failed: {error}")

                elif curr_drone_state == drone_state.DASHING.value:
                    # if (current_state == drone_state.FOLLOWING.value 
                    #     or current_state == drone_state.SEARCH.value
                    #     # or current_state == drone_state.EP.value
                    #     or current_state == drone_state.EPG.value
                    #     or current_state == drone_state.EL.value) and errors is not None:
                    #     # Following and Dashing are DEPRECATED
                    #     offsetedErrors = np.array(errors[:4])
                    #     offsetedErrors[2] += YOFFSETDASH
                    #     offsetedErrors[3] -= ZOFFSETDASH
                    #     await resetPIDDash(offsetedErrors)
                    #     current_state = curr_drone_state
                    #     lockIn = False
                    #     lockX = False
                    #     doLockX = True
                    if errors is not None:
                    
                        # Transition dashing command received to Guidance Dash
                        curr_drone_state = drone_state.GUIDANCEDASH.value
                        offsetedErrors = np.array(errors[:4])
                        offsetedErrors[2] += YOFFSETDASH
                        offsetedErrors[3] -= ZOFFSETDASH
                        current_state = curr_drone_state

                elif curr_drone_state == drone_state.FOLLOWING.value:
                    if current_state == drone_state.DASHING.value and errors is not None:
                        offsetedErrors = np.array(errors[:4])
                        offsetedErrors[2] += YOFFSETFOLLOW
                        offsetedErrors[3] -= ZOFFSETFOLLOW
                        await resetPIDFollow(offsetedErrors)
                        current_state = curr_drone_state

                elif curr_drone_state == drone_state.EL.value:
                    if (is_start_eagle_line and current_state != drone_state.FOLLOWING.value) and (current_state != drone_state.DASHING.value):
                        tower_point = msg['Point1']
                        end_eagle_line_point = msg['Point2']
                        # Check if current state not EL because we don't want to keep calling get_displacement_to_line_segment function repeatedly
                        if (current_state is not drone_state.EL.value and drone_point is not None and get_displacement_to_line_segment(tower_point, end_eagle_line_point, drone_point, is_reach_eagle_line)[0] is not None):
                            current_state = drone_state.EL.value

                elif curr_drone_state == drone_state.EP.value:
                    if (is_start_eagle_point and current_state != drone_state.FOLLOWING.value) and (current_state != drone_state.DASHING.value) and (current_state != drone_state.GUIDANCEDASH.value):
                        eagle_point = msg['Point']
                        drone_point_adjusted = [drone_point[0], drone_point[1], drone_point[2 + int(msg.get('HType', 0))]]
                        # Check if current state not EP because we don't want to keep calling get_displacement_to_line_segment function repeatedly
                        if (current_state is not drone_state.EP.value and drone_point is not None and get_displacement_to_point(drone_point_adjusted, eagle_point, replay_buffer)[0] is not None):
                            current_state = drone_state.EP.value
                
                elif curr_drone_state == drone_state.EPG.value:
                    if (is_start_eagle_point_guard and current_state != drone_state.FOLLOWING.value) and (current_state != drone_state.DASHING.value) and (current_state != drone_state.GUARDDASH.value):
                        if process_epg_message(msg, drone_point):
                            current_state = drone_state.EPG.value

                elif curr_drone_state == drone_state.RESETEL.value:
                    is_reach_eagle_line = False
                    is_start_eagle_line = False
                    current_state = drone_state.SEARCH.value

                elif curr_drone_state == drone_state.STARTEL.value:
                    is_start_eagle_line = True

                elif curr_drone_state == drone_state.RESETEP.value:
                    is_start_eagle_point = False
                    current_state = drone_state.SEARCH.value

                elif curr_drone_state == drone_state.STARTEP.value:
                    is_start_eagle_point = True

                elif curr_drone_state == drone_state.RESETEPG.value:
                    epg_state.reset()
                    is_start_eagle_point_guard = False
                    current_state = drone_state.SEARCH.value

                elif curr_drone_state == drone_state.STARTEPG.value:
                    is_start_eagle_point_guard = True

                elif curr_drone_state == drone_state.DETECTIONON.value:
                    detection_enabled = True
                    print("Detection enabled")
                
                elif curr_drone_state == drone_state.DETECTIONOFF.value:
                    detection_enabled = False
                    print("Detection disabled")

            # print(drone_point)
            if info is not None:
                drone_point = [info["lat"], info["long"], info["altAGL"], info["altAMSL"]]
                # print(drone_point)
        #Output
            if is_offboard:
                # print("current_state: ", current_state)
                if offsetedErrors is not None:
                    speeds = pid.pid_control(pid_error, pid_signal,offsetedErrors)
                    global filtered_log_width_rate
                    filtered_log_width_rate = pid.filter_derivative(log_width, filtered_log_width_rate, current_log_width, W=5)
                    global filtered_distance_history
                    filtered_distance_history = pid.filter_derivative(distance_history, filtered_distance_history, last_target_range, W=1)
                    global filtered_px_rate_history_x, filtered_px_rate_history_y
                    filtered_px_rate_history_x = pid.filter_derivative(px_history_x, filtered_px_rate_history_x, px_error[0], W=10000000)
                    filtered_px_rate_history_y = pid.filter_derivative(px_history_y, filtered_px_rate_history_y, px_error[1], W=10000000)
                    global prev_yaw

                try:
                    if current_state == drone_state.SEARCH.value:
                        #droneStatus = "Search"
                        # print("scurrent earching")
                        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, SEARCH_YAW * np.sign(px_error[0])))
                    elif current_state == drone_state.FOLLOWING.value:
                        #droneStatus ="Follow"
                        if speeds is not None and len(speeds) >= 4:
                            #Additional correction
                            speedY = speeds[1] + px_error[1] * 0.002
                            if last_target_range > 15: #Further than 15m from target - px correct to catch fast targets
                                speedFwd = speeds[0] + px_error[1] * 0.02
                            else: #Don't correct <15m as target is likely too low, not too fast.
                                speedFwd = speeds[0] + px_error[1] * 0
                            corrected_speeds = (speedFwd,speedY,speeds[2],speeds[3])
                            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(speedFwd, 0.0, speedY, speeds[3])) #fwd, right, down, yaw
                        else:
                            print("Invalid speeds data")
                    elif current_state == drone_state.DASHING.value:
                        # Deprecated
                        #droneStatus ="Dash"
                        # Pixel correction rules
                        if lockIn:
                            speedFwd = lockspeedFwd
                            speedY = lockspeedY + px_error[1] * 0.01
                        else:
                            speedFwd = speeds[0] + px_error[1] * 0.01 #Positive = drone needs to speed up in fwd direction
                            speedY = speeds[1] + px_error[1] * 0.01

                        if lockX: #Straight moving target. Lock X speed to prevent oscillation
                            speedX = lockspeedX + px_error[0] * 0.01
                        else: #Circular moving target within 3m. Continue using PID.
                            speedX = speeds[2] + px_error[0] * 0.0

                        corrected_speeds = (speedFwd,speedY,speedX,speeds[3])
                        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(speedFwd, speedX, speedY, speeds[3])) #fwd, right, down, yaw

                        if (px_error[0] > 200 or px_error[0] < -200) and doLockX: #For circular moving targets, don't lock X
                            doLockX = False

                        if pid_error[0,3] < 3: #When target is within 3m, lock the Y velocity. This is to prevent close up shakiness due to incaccurate CV at close range
                            print("<3")
                            if not lockIn and px_error[1] < 16 and px_error[1] > -16:
                                print("locked on")
                                lockIn = True
                                speedMultiplier = min(1.6, VZMAX / speedFwd) #1.6x current speed, capped at VZMAX
                                lockspeedY = speedY * speedMultiplier #Give a boost to hit max speed
                                lockspeedFwd = speedFwd * speedMultiplier

                            if doLockX and not lockX and px_error[0] < 10 and px_error[0] > -10:
                                lockX = True
                                lockspeedX = speedX
                    elif current_state == drone_state.EL.value:
                        if (drone_point is not None and is_start_eagle_line):
                            try:
                                target_point, yaw_deg, is_reach_eagle_line = get_displacement_to_line_segment(tower_point, end_eagle_line_point, drone_point, is_reach_eagle_line)
                                
                                # Reached the end of the eagle line, return back to tower or just search on the spot
                                if (target_point is None):
                                    print("i am out")
                                    is_start_eagle_line = False
                                    current_state = drone_state.SEARCH.value
                            except Exception as e:
                                print(e)
                            
                            try:
                                async with timeout(1):
                                    if (target_point is not None):
                                        NSpeed, ESpeed, USpeed = EAGLELINESPEED * target_point/vector_magnitude(target_point)
                                        try:
                                            #await drone.offboard.set_position_global(PositionGlobalYaw(lat, long, max(5, alt), yaw_deg, PositionGlobalYaw.AltitudeType(0)))
                                            await drone.offboard.set_velocity_ned(VelocityNedYaw(NSpeed, ESpeed, -USpeed, yaw_deg))
                                        except Exception as e:
                                            print(e)
                            except asyncio.TimeoutError:
                                print("Timeout")
                    elif current_state == drone_state.EP.value:
                        if (msg is not None and drone_point is not None and is_start_eagle_point):
                            try:
                                eagle_point = msg['Point']
                                drone_point_adjusted = [drone_point[0], drone_point[1], drone_point[2 + int(msg.get('HType', 0))]]
                                target_point, yaw_deg, replay_buffer = get_displacement_to_point(drone_point_adjusted, eagle_point, replay_buffer)
                            except Exception as e:
                                print(e)
                            
                            try:
                                async with timeout(1):
                                    if (target_point is not None):
                                        target_point[2] = target_point[2] - EP_ALT_OFFSET
                                        NSpeed, ESpeed, USpeed  = calculate_speed_gain(target_point)
                                        try:
                                            await drone.offboard.set_velocity_ned(VelocityNedYaw(NSpeed, ESpeed, USpeed, yaw_deg))
                                        except Exception as e:
                                            print(e)
                            except asyncio.TimeoutError:
                                print("Timeout")
                    elif current_state == drone_state.EPG.value:
                        if (drone_point is not None and is_start_eagle_point_guard):
                            try:
                                if msg is not None:
                                    process_epg_message(msg, drone_point)
                                
                                drone_point_adjusted = [drone_point[0], drone_point[1], drone_point[2 + epg_state.height_type]]
                                movement = calculate_epg_movement(drone_point_adjusted)
                                if movement:
                                    n_speed, e_speed, u_speed, yaw = movement
                                    async with timeout(1):
                                        await drone.offboard.set_velocity_ned(
                                            VelocityNedYaw(n_speed, e_speed, u_speed, yaw)
                                        )
                            except asyncio.TimeoutError:
                                print("EPG operation timeout")
                            except Exception as e:
                                print(f"EPG operation failed: {e}")
                    elif current_state == drone_state.GUARDDASH.value and px_history_x[1] != 0: # skip guard dash for 1 frame to get target drone velocity
                        if speeds is not None and len(speeds) >= 4 : 
                            # derive distance
                            print("3D errors:", offsetedErrors)
                            print("pixel error:" ,px_error)
                            print(filtered_distance_history)
                            closing_speed=max(0, -filtered_distance_history[0][0])
                            # closing_speed = 10
                            print('predicted target speed', closing_speed)
                            speedX = (math.atan(px_error[0]/pid.FOCAL_LENGTH_PIXELS) * (EAGLEGUARD_HORIZ_DASH_PROPORTIONAL_GAIN*closing_speed))
                            speedY = (math.atan(px_error[1]/pid.FOCAL_LENGTH_PIXELS) * (EAGLEGUARD_VERT_DASH_PROPORTIONAL_GAIN*closing_speed))
                            corrected_speeds = (speeds[0],speedY,speedX,speeds[3])
                            print("speeds", corrected_speeds)
                            print()
                            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, speedX, speedY, 0.0)) #fwd, right, down, yaw

                            # # derive angluar velocity
                            # print("3D errors:", offsetedErrors)
                            # print("pixel error:" ,px_error)
                            # print('predicted target speed', closing_speed)
                            # print('filtered_px_rate_history', filtered_px_rate_history_x, filtered_px_rate_history_y)
                            # print('px_history', px_history_x, px_history_y)
                            # # closing_speed=max(0, -filtered_distance_history[0][0])
                            # # closing_speed=10
                            # print('predicted target speed', closing_speed)
                            # # speedY = (math.atan(px_error[1]/pid.FOCAL_LENGTH_PIXELS) * (EAGLEGUARD_DASH_PROPORTIONAL_GAIN*closing_speed))
                            # speedX = (last_target_range * EAGLEGUARD_HORIZ_DASH_PROPORTIONAL_GAIN * (filtered_px_rate_history_x[0][0] / (px_history_x[0][0]**2 + pid.FOCAL_LENGTH_PIXELS**2)) * pid.FOCAL_LENGTH_PIXELS)
                            # speedY = (last_target_range * EAGLEGUARD_VERT_DASH_PROPORTIONAL_GAIN * (filtered_px_rate_history_y[0][0] / (px_history_y[0][0]**2 + pid.FOCAL_LENGTH_PIXELS**2)) * pid.FOCAL_LENGTH_PIXELS)
                            # corrected_speeds = (speeds[0],speedY,speedX,speeds[3])
                            # print("speeds", corrected_speeds)
                            # print()
                            # await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, speedX, speedY, 0.0)) #fwd, right, down, yaw
                    elif current_state == drone_state.GUIDANCEDASH.value: 
                        if speeds is not None and len(speeds) >= 4 and px_history_x[1] != 0: #skip first frame

                            # # V1 yaw, y, fwd
                            # print("3D errors:", offsetedErrors)
                            # print("pixel error:" ,px_error)
                            # print('px_history', px_history_x, px_history_y)
                            # speedY = (math.atan(px_error[1]/pid.FOCAL_LENGTH_PIXELS) * (EAGLEPOINT_DASH_PROPORTIONAL_GAIN*EAGLEPOINT_DASH_VELOCITY))
                            # yaw_rate = EAGLEPOINT_DASH_PROPORTIONAL_GAIN * math.degrees((filtered_px_rate_history_x[0][0] / (px_history_x[0][0]**2 + pid.FOCAL_LENGTH_PIXELS**2)) * pid.FOCAL_LENGTH_PIXELS)
                            # # speedY = 2*(last_target_range * EAGLEPOINT_DASH_PROPORTIONAL_GAIN * (filtered_px_rate_history_y[0][0] / (px_history_y[0][0]**2 + pid.FOCAL_LENGTH_PIXELS**2)) * pid.FOCAL_LENGTH_PIXELS)
                            # corrected_speeds = (yaw_rate,speedY,0,EAGLEPOINT_DASH_VELOCITY) # yaw, down, right, fwd
                            # print("speeds", corrected_speeds)
                            # print()
                            # await drone.offboard.set_velocity_body(VelocityBodyYawspeed(EAGLEPOINT_DASH_VELOCITY, 0, speedY, yaw_rate)) #fwd, right, down, yaw
                            
                            # # V2 PPN X, Y, fwd
                            # print("3D errors:", offsetedErrors)
                            # print("pixel error:" ,px_error)
                            # print('px_history', px_history_x, px_history_y)
                            # speedY = (math.atan(px_error[1]/pid.FOCAL_LENGTH_PIXELS) * (EAGLEPOINT_DASH_PROPORTIONAL_GAIN*EAGLEPOINT_DASH_VELOCITY))
                            # speedX = (math.atan(px_error[0]/pid.FOCAL_LENGTH_PIXELS) * (EAGLEGUARD_DASH_PROPORTIONAL_GAIN*EAGLEPOINT_DASH_VELOCITY))
                            # # yaw_rate = EAGLEPOINT_DASH_PROPORTIONAL_GAIN * math.degrees((filtered_px_rate_history_x[0][0] / (px_history_x[0][0]**2 + pid.FOCAL_LENGTH_PIXELS**2)) * pid.FOCAL_LENGTH_PIXELS)
                            # corrected_speeds = (0,speedY,speedX,EAGLEPOINT_DASH_VELOCITY) # yaw, down, right, fwd
                            # print("speeds", corrected_speeds)
                            # print()
                            # await drone.offboard.set_velocity_body(VelocityBodyYawspeed(EAGLEPOINT_DASH_VELOCITY, speedX, speedY, 0)) #fwd, right, down, yaw

                            # V3 PPN X, Y, fwd
                            print("3D errors:", offsetedErrors)
                            print("pixel error:" ,px_error)
                            print('px_history', px_history_x, px_history_y)
                            LOS_camera_vector = np.array([px_history_x[0][0] , px_history_y[0][0], pid.FOCAL_LENGTH_PIXELS]).T # delta x, delta y, focal length
                            camera_to_body_transformation = np.array([[1,0,0], [0,math.cos(pid.PITCH), -math.sin(pid.PITCH)], [0, math.sin(pid.PITCH), math.cos(pid.PITCH)]])
                            LOS_body_vector = camera_to_body_transformation @ LOS_camera_vector 
                            LOS_rate_body_vector = np.array([filtered_px_rate_history_x[0][0], math.cos(pid.PITCH)*filtered_px_rate_history_y[0][0], math.sin(pid.PITCH)*filtered_px_rate_history_y[0][0]]) # right, down, fwd
                            omega = np.cross(LOS_body_vector, LOS_rate_body_vector) / np.dot(LOS_body_vector, LOS_body_vector)

                            # --------- APN AUGMENTATION (SAFE) ---------

                            dt = 1.0 / pid.TELEMETRY_RATE_IN_HZ

                            omega_dot = (omega - previous_omega) / (dt + 1e-6)
                            previous_omega[:] = omega

                            AUG_GAIN = 0.15        # start small (0.05–0.2)
                            AUG_LIMIT = 1.0       # hard clamp (IMPORTANT)

                            a_aug = AUG_GAIN * omega_dot
                            a_aug = np.clip(a_aug, -AUG_LIMIT, AUG_LIMIT)


                            if previous_velocity is None or np.ndim(previous_velocity) == 0:
                                fwd_velocity = math.sqrt(NSpeed**2 + ESpeed**2)
                                down_velocity = -USpeed
                                previous_velocity = np.array([0.0, down_velocity, fwd_velocity], dtype=float)

                            omega = np.asarray(omega, dtype=float).reshape(3,)
                            previous_velocity = np.asarray(previous_velocity, dtype=float).reshape(3,)

                            # ---- NOW SAFE ----
                            a_pn = EAGLEPOINT_DASH_PROPORTIONAL_GAIN * np.cross(omega, previous_velocity)
                            
                            #current_velocity = previous_velocity + (1/pid.TELEMETRY_RATE_IN_HZ) * EAGLEPOINT_DASH_PROPORTIONAL_GAIN * np.cross(omega, previous_velocity)
                            # --- Total acceleration ---
                            a_total = a_pn + a_aug
                            
                            # --- Integrate ---
                            current_velocity = previous_velocity + dt * a_total


                            current_velocity = current_velocity * EAGLEPOINT_DASH_VELOCITY / math.sqrt(current_velocity[0]**2 + current_velocity[1]**2 + current_velocity[2]**2)
                            corrected_speeds = (0, min(VYDOWNMAX, max(-VYUPMAX, current_velocity[1])), min(VXMAX, current_velocity[0]), min(VZMAX, current_velocity[2])) # yaw, down, right, fwd
                            speedY = corrected_speeds[1]
                            speedX = corrected_speeds[2]
                            speedFwd = corrected_speeds[3]
                            previous_velocity = np.array([speedX, speedY, speedFwd], dtype=float)
                            print("speeds", corrected_speeds)
                            print()
                            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(speedFwd, speedX, speedY, 0)) #fwd, right, down, yaw

                #print(droneStatus)
                except OffboardError as error:
                    print(f"Stopping Offboard Mode Due to set velocity Error: {error}")
                    current_state = drone_state.SEARCH.value
                    await drone.offboard.stop()
                    is_offboard = False    # puts drone in Hold mode
        #Message Out
            drone_meta = {
                'droneStatus': current_state,
                'isDroneDetected': offsetedErrors is not None 
            }
            for subscriber in MQTT_send_topics:
                subscriber.push(drone_meta)

            # send the bounding box data to the ground station
            for subscriber in RC3_send_topics:
                subscriber.push({
                    "Target Coordinates": target_pixel_coords,
                    "Errors": errors,
                    "Speeds": corrected_speeds,
                    "Closing Speed": closing_speed,
                    "Elapsed Time": time() - ptime,
                })
            
            ptime = time()

def wrapped_flight_compute(topics_in, topics_out):
    asyncio.run(main(topics_in, topics_out))
