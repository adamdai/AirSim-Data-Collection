"""Log vehicle pose

"""

import numpy as np
import airsim
import time

client = airsim.MultirotorClient()
client.confirmConnection()

dt = 0.1
states = []
collision_count = 0

try:
    while True:
        pose = client.simGetVehiclePose()
        x, y, z = pose.position.x_val, pose.position.y_val, pose.position.z_val
        position = np.array([x, y, z])
        states.append(position)
        print(f"x = {x:.2f}, y = {y:.2f}, z = {z:.2f}")
        time.sleep(dt)

except KeyboardInterrupt:
    print("Saving data")
    timestamp = time.strftime("%Y%m%d-%H%M")
    np.savez(f'../data/states_{timestamp}.npz', states=states)
    print("Interrupted by user, shutting down")
    client.enableApiControl(False)
    exit()