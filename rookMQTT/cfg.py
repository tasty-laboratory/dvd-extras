from enum import IntEnum

# enums
class Os(IntEnum):
    UBUNTU = 0
    WINDOWS = 1
    BLUE = 2
    GAZEBO = 3
    AIRSIM = 4

class DroneState(IntEnum):
    UNINITIALISED = 0 ## UNUSED ##
    SEARCH = 1 # set_velocity_body
    TRACK = 2 ## UNUSED ##
    FOLLOWING = 3 # set_velocity_body ## UNUSED ##
    DASHING = 4 # set_velocity_body
    HOLDSEARCH = 5 ## UNUSED ##
    ONBOARD = 6 # onboard
    LOW = 7 ## UNUSED ##
    EAGLELINE = 8 ## UNUSED ##
    OFFBOARD = 9 # offboard
    FALSE_POS = 10 ## UNUSED ##
    LAUNCH = 11 # launch
    LAND = 12 # land
    EL = 13 # set_velocity_ned
    RESETEL = 14
    STARTEL = 15
    EP = 16 # Eagle Point
    RESETEP = 17
    STARTEP = 18
    EPG = 19 # Eagle Point (Predictive Hold)
    RESETEPG = 20
    STARTEPG = 21
    DETECTIONON = 22 # Toggles the boolean flag auto_folllow_enabled
    DETECTIONOFF = 23 # Toggles the boolean flag auto_folllow_enabled
    GUARDDASH = 24 # State for dashing into targets when detected during guard 
    GUIDANCEDASH = 25 # State for dramatic dash after guidance
    
# shared
INPUT_FRAME_WIDTH = 1920
INPUT_FRAME_HEIGHT = 1080
INPUT_FPS = 30

USE_BBOX_CLIENT = False

# OS = Os.GAZEBO
# OS = Os.UBUNTU
OS = Os.AIRSIM

#MQTT Constants To Send
DRONEMODEL = "Holybro X500"
if OS == Os.GAZEBO:
    #SITL
    # MQTT_BROKER_IP = "192.168.0.181" # Using Rovio
    MQTT_BROKER_IP = "localhost" 
    DRONECALLSIGN = "O4"
    # MAVSDK_SERVER_IP = "192.168.0.181" # Using Rovio
    MAVSDK_SERVER_IP = "localhost"
    # MAVSDK_SERVER_IP = "10.168.10.85"
    RTSP_URL = f"rtsp://{MAVSDK_SERVER_IP}:8554/annotated" # RTSP url for gazebo stream
    MJPEG_URL = f"http://localhost:8080/annotated"

elif OS == Os.UBUNTU:
    MQTT_BROKER_IP = "192.168.144.50"
    DRONECALLSIGN = "O4"
    MAVSDK_SERVER_IP = "127.0.0.1" #"192.168.144.25"
    RTSP_URL = f"rtsp://192.168.144.50:8554/annotated"
    MJPEG_URL = f"http://192.168.144.60:8080/annotated"

elif OS == Os.AIRSIM:
    MQTT_BROKER_IP = "localhost" 
    DRONECALLSIGN = "O4"
    MAVSDK_SERVER_IP = "localhost"
    RTSP_URL = f"rtsp://192.168.144.50:8554/mystream" 
    MJPEG_URL = f"http://localhost:8080/annotated"
    WEBRTC_URL = f"ws://192.168.144.106:80/annotated"
    
# for display
BBOX_COLOUR = (0,0,255) # BGR

DISPLAY_FRAME_WIDTH = 1920
DISPLAY_FRAME_HEIGHT = 1080