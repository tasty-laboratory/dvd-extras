import math
import numpy as np


TELEMETRY_RATE_IN_HZ = 30.0 # rate of sending of target_info

# camera stats SIYI A8 mini
HFOV = math.radians(81) # radians
RES = [1920, 1080]
CENTER = [RES[0]/2, RES[1]/2+225]
FOCAL_LENGTH_PIXELS = (RES[0] / 2) / math.tan(HFOV / 2)
PITCH = math.radians(21.8) # Camera's pitch above horizon, radians
# PITCH = math.radians(11.3) # Camera's pitch above horizon, radians
COSP = math.cos(PITCH)
SINP = math.sin(PITCH)

# CV params
CVCORRECTIONFACTOR = 1
WIDTHPIXELDEADZONE = 0 * CVCORRECTIONFACTOR
DRONE_WIDTH = 0.6 # meters

# Speed Limit
VZMAX = 14 #Forward
VYUPMAX = 8 #Vertical Up
VYDOWNMAX = 3 #Vertical Down
VXMAX = 14 #Horizontal
VYAWMAX = 45 #Yaw

# PID variables (Ensure last 3 columns are the same to Dash in a straight line)
K = np.array([
    [20.0, 0.28, 0.28, 0.28], # KP (yaw,x,y,z)
    [1.5, 0.0384, 0.0384, 0.0384], # KI (yaw,x,y,z)
    [0.0, 0.0, 0.0, 0.0]  # KD (yaw,x,y,z)
])

# filter coefficient
N = 1000000
Ts = 1.0 / TELEMETRY_RATE_IN_HZ
W = 1000000 # lowpass cutoff frequency for log width

# ProNav Gain
NPRO = 3

a = np.array([(1.0 + N * Ts), -(2.0 + N * Ts), 1.0]) # (3, )
ku = a[1:] / a[0] # (2, )

b = np.array([
    [a[0], Ts * a[0], N],
    [a[1], -Ts, -2 * N],
    [1.0, 0.0, N]
]) @ K
# (3, 3) @ (3, 4) -> (3, 4)

ke = b / a[0] # (3, 4), same as b


def get_range_in_drone_x_axis(bbox):
    if bbox is None:
        return None
    bbox_width = max(WIDTHPIXELDEADZONE,bbox[2])
    distance_meters = (DRONE_WIDTH * CVCORRECTIONFACTOR*FOCAL_LENGTH_PIXELS) / bbox_width
    
    return distance_meters, math.log(bbox_width)

def calculate_errors(bbox, gimbalYaw):
    Yaw = math.radians(gimbalYaw)
    COSY = math.cos(Yaw)
    SINY = math.sin(Yaw)
    if bbox is None:
        return None
    # pixel errors of top center of bbox center from frame center + estimated range from bbox width
    camera_x = (bbox[0] + bbox[2] / 2 - CENTER[0])
    camera_y = (bbox[1] + bbox[3] / 2 - CENTER[1])
    range_in_drone_x_axis, log_width = get_range_in_drone_x_axis(bbox)

    error_camera_coordinates = np.array([
        camera_x * range_in_drone_x_axis / FOCAL_LENGTH_PIXELS, # meters to translate right until target is centered from camera's POV, positive means target is on the right
        camera_y * range_in_drone_x_axis / FOCAL_LENGTH_PIXELS, # meters to translate down until target is centered from camera's POV, positive means target is below
        range_in_drone_x_axis # meters to translate forward until targets is reached from camera POV
    ])

    error_drone_coordinates = np.array([
        COSY*error_camera_coordinates[0] - SINP*SINY*error_camera_coordinates[1] - COSP*SINY*error_camera_coordinates[2],
        COSP*error_camera_coordinates[1] - SINP*error_camera_coordinates[2],
        SINY*error_camera_coordinates[0] + SINP*COSY*error_camera_coordinates[1] + COSP*COSY*error_camera_coordinates[2]
    ])

    return np.array([
        error_drone_coordinates[0]/error_drone_coordinates[2], # tangent of the horizontal angle (rad) to yaw until target is centered, positive means target is on the right, for small angles tangent is roughly identity function 
        error_drone_coordinates[0], # meters to translate right until target is centered, positve means target is on the right
        error_drone_coordinates[1], # meters to translate down until target is coaltitude with self, positive means target is below
        error_drone_coordinates[2], # meters away from target
        camera_x,                    # number of horizontal pixels target is from the center of the camera, +ve means target is right
        camera_y,                     # number of vertical pixels target is from the center of the camera, +ve  means target is down
        log_width
        ])


def pid_control(pid_error, pid_signal, error):
    pid_error[[1, 2], :] = pid_error[[0, 1], :]
    pid_signal[[1, 2], :] = pid_signal[[0, 1], :]


    pid_error[0] = error

    keE = np.einsum("ij,ij->j", ke, pid_error) # equivalent to np.diag(ke.T @ pid_error)
    pid_signal[0] = keE - ku @ pid_signal[[1, 2], :]
    
    #limit pid_signal to the speed limit of the drone. Important to prevent integral windup.
    pid_signal[0, 3] = min(max(pid_signal[0, 3],-VZMAX),VZMAX) # Forward Speed in m/s, +ve is forward
    pid_signal[0, 2] = min(max(pid_signal[0, 2],-VYUPMAX),VYDOWNMAX) # Down Speed in m/s, +ve is downward
    pid_signal[0, 1] = min(max(pid_signal[0, 1],-VXMAX),VXMAX) # Right Speed in m/s, +ve is right
    pid_signal[0, 0] = min(max(pid_signal[0, 0],-VYAWMAX),VYAWMAX) # in deg/s, +ve is clockwise
    #print(pid_signal[0, 2])
    #print(f"bbox_size: {bbox[2]:>8.3f}", f"x_offset_ratio: {camera_x:>8.3f}", f"y_offset_ratio: {camera_y:>8.3f}", f"range: {range_in_drone_x_axis:>8.3f}", f"forward_speed: {forward_speed/SCALE:>8.3f}", f"down_speed: {down_speed/SCALE:>8.3f}", f"yaw_speed: {yaw_speed/SCALE:>8.3f}")

    return (pid_signal[0, 3], pid_signal[0, 2], pid_signal[0, 1], pid_signal[0, 0])

def filter_derivative(history, derivative_history, current_value, W=W):   
    history[1] = history[0]
    derivative_history[1] = derivative_history[0]
    history[0] = current_value

    #derivative_history[0] = derivative_history[1]*(2-Ts*W)/(2+Ts*W) + history[0]*2*W/(2+Ts*W) - history[1]*2*W/(2+Ts*W) 
    derivative_history[0] = derivative_history[1]/(1+Ts*W) + (history[0]-history[1])*W/(1+Ts*W) 
    # print('log width', history, 'filtered', derivative_history,'current',current_value)
    return derivative_history

def proportional_navigation(errorNED, errorRateNED, errorYEDN):
    errorNED[1] = errorNED[0]
    errorRateNED[1] = errorRateNED[0]
    errorNED[0] = np.array([
        errorYEDN[3], # meters to translate North until target is colatitude with self
        errorYEDN[1], # meters to translate East until target is colongitude with self
        errorYEDN[2]  # meters to translate Down until target is coaltitude with self
        ])
    errorRateNED[0] = errorRateNED[1]/(1+N*Ts) + (N/(1+N*Ts))*(errorNED[0]- errorNED[1])
    

def augmented_proportional_navigation(
        LOS_body_vector,
        LOS_rate_body_vector,
        ownship_velocity_body,
        target_accel_body_est,
        N_gain=3):

    # Normalized LOS
    LOS_norm = np.linalg.norm(LOS_body_vector)
    if LOS_norm == 0:
        return np.zeros(3)
    L_hat = LOS_body_vector / LOS_norm

    # 1) Standard PN acceleration: a_PN = N * (Vo × λ̇)
    #    (In body frame, LOS_rate_body_vector is λ̇ approx)
    a_PN = N_gain * np.cross(ownship_velocity_body, LOS_rate_body_vector)

    # 2) Compute component of target acceleration perpendicular to LOS
    #    a_T_perp = a_T - (a_T · L̂) L̂
    a_T = target_accel_body_est
    a_T_parallel = np.dot(a_T, L_hat) * L_hat
    a_T_perp = a_T - a_T_parallel  # APN augmentation term

    # 3) Total APN commanded acceleration
    a_cmd = a_PN + a_T_perp

    return a_cmd


# PID controller logic for eagle point prediction hold
def pid_distance(target_position, current_position, dt, state, 
                 Kp=0.4, Ki=0.0, Kd=0.2, max_speed=3.0):
    # Convert lat/lon to meters
    north_err = (target_position[0] - current_position[0]) * 111320
    east_err = (target_position[1] - current_position[1]) * \
               (40075000 * np.cos(np.radians(current_position[0])) / 360)
    down_err = current_position[2] - target_position[2]  # NED: +ve is down

    error = np.array([north_err, east_err, down_err])

    # Update state
    state['integral'] += error * dt
    derivative = (error - state['last_error']) / dt if dt > 0 else np.zeros(3)
    state['last_error'] = error

    # PID output
    velocity = Kp * error + Ki * state['integral'] + Kd * derivative

    # Clip to max speed
    speed = np.linalg.norm(velocity)
    if speed > max_speed:
        velocity = velocity / speed * max_speed

    return velocity
