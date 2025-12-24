import cosysairsim as airsim
import math

client = airsim.MultirotorClient(ip='192.168.144.67')
vehicle_name = "O4"

def euler_to_quaternion(pitch, roll, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return airsim.Quaternionr(x, y, z, w)

def quaternion_to_euler(q):
    w, x, y, z = q.w_val, q.x_val, q.y_val, q.z_val
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def quaternion_inverse(q):
    return airsim.Quaternionr(-q.x_val, -q.y_val, -q.z_val, q.w_val)

def quaternion_multiply(q1, q2):
    w = q1.w_val*q2.w_val - q1.x_val*q2.x_val - q1.y_val*q2.y_val - q1.z_val*q2.z_val
    x = q1.w_val*q2.x_val + q1.x_val*q2.w_val + q1.y_val*q2.z_val - q1.z_val*q2.y_val
    y = q1.w_val*q2.y_val - q1.x_val*q2.z_val + q1.y_val*q2.w_val + q1.z_val*q2.x_val
    z = q1.w_val*q2.z_val + q1.x_val*q2.y_val - q1.y_val*q2.x_val + q1.z_val*q2.w_val
    return airsim.Quaternionr(x, y, z, w)

while True:
    drone_pose = client.simGetVehiclePose(vehicle_name=vehicle_name)
    drone_orientation = drone_pose.orientation

    # Extract drone's yaw
    _, _, drone_yaw = quaternion_to_euler(drone_orientation)

    # Desired global camera orientation (pitch=21.8°, roll=0°, yaw=drone_yaw)
    camera_pitch = math.radians(21.8)
    camera_roll = 0
    camera_yaw = drone_yaw
    desired_global_quat = euler_to_quaternion(camera_pitch, camera_roll, camera_yaw)

    camera_pose = airsim.Pose(airsim.Vector3r(0,0,0), desired_global_quat)
    client.simSetCameraPose(0, pose=camera_pose, vehicle_name=vehicle_name)
