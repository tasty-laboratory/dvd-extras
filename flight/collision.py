import cosysairsim as airsim
import time

client = airsim.MultirotorClient(ip='192.168.144.67')
hunter = "O4"
target = "target"

def check_collision(vehicle_name, target_name):
    collision_info = client.simGetCollisionInfo(vehicle_name=vehicle_name)
    if collision_info.has_collided and collision_info.object_name == target_name:
        print(collision_info)
        return collision_info.has_collided
    return False

collision_count = 0
while True:
    hunter_collided = check_collision(hunter, target)

    if hunter_collided:
        print(f"Dash {collision_count} successful")
        collision_count += 1
