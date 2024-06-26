"""Log vehicle pose

"""

import numpy as np
import airsim
import time

client = airsim.CarClient()
client.confirmConnection()

dt = 0.1
states = []
accelerations = []
collision_count = 0
velocities = []

try:
    while True:
        pose = client.simGetVehiclePose()
        car_state = client.getCarState()
        #print(car_state)
        x, y, z = pose.position.x_val, pose.position.y_val, pose.position.z_val
        x_acc =  car_state.kinematics_estimated.linear_acceleration.x_val
        y_acc =  car_state.kinematics_estimated.linear_acceleration.y_val
        z_acc =  car_state.kinematics_estimated.linear_acceleration.z_val
        acceleration =  np.array([x_acc, y_acc, z_acc])

        x_vel = car_state.kinematics_estimated.linear_velocity.x_val
        y_vel = car_state.kinematics_estimated.linear_velocity.y_val
        z_vel = car_state.kinematics_estimated.linear_velocity.z_val
        velocity = np.array([x_vel, y_vel, z_vel])

        #position = np.array([x, y, z])
        #states.append(position)
        accelerations.append(acceleration)
        #print(f"x = {x:.2f}, y = {y:.2f}, z = {z:.2f}")
        #print(f"x_acc = {x_acc:.2f}, y_acc = {y_acc:.2f}, z_acc = {z_acc:.2f}")
        print(f"x_vel = {x_vel:.6f}, y_vel = {y_vel:.6f}, z_vel = {z_vel:.6f}")

        time.sleep(dt)

except KeyboardInterrupt:
    print("Saving data")
    timestamp = time.strftime("%Y%m%d-%H%M")
    np.savez(f'../data/states_{timestamp}.npz', states=states)
    np.savez(f'../data/accelerations_{timestamp}.npz', accelerations=accelerations)

    print("Interrupted by user, shutting down")
    client.enableApiControl(False)
    exit()