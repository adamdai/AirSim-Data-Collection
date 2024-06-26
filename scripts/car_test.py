"""Leader and follow cars taking images of each other
   
   - Car 2 is leader, car 1 is follower
   - Car 1 tracks a goal pose that is a fixed distance behind car 2

Notes
-----
    Positive steer turns right, negative turns left

Settings: 'settings_two_car.json'

"""

import airsim_data_collection.common.setup_path
import airsim
import os
import numpy as np
import time
import shutil
import argparse

from airsim_data_collection.sensors.sensor_handler import SensorHandler
from airsim_data_collection.utility.convert_pose import quaternion_to_eul, angle_diff, airsimpos2np
import matplotlib.pyplot as plt



## -------------------------- MAIN ------------------------ ##
if __name__ == "__main__":

    # Connect to client
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True)

    car_controls = airsim.CarControls()

    time.sleep(3)
    # brake the car
    car_controls.brake = 1
    car_controls.throttle = 0
    client.setCarControls(car_controls)
    # wait until car is stopped
    time.sleep(3)

    # png_image = client.simGetImage("BirdsEyeCamera", airsim.ImageType.Scene)
    # print("captured image")
    # # save image
    # airsim.write_file(os.path.normpath('C:/Users/Adam/Documents/AirSim/image.png'), png_image)

    # get vehicle position
    car_state = client.getCarState()
    print("car state: %s" % car_state)

    acceleration_list = np.zeros((10,10,3))
    velocity_list = np.zeros((10,10,3))
    throttle_values = []

    try:
        print("driving routes")
        for i, throttle in enumerate(np.linspace(0.1, 1.0, 10)):
            print("executing throttle: " + str(throttle))
            car_controls.steering = 0.0  #0.2 * np.sin(0.01 * count)
            car_controls.throttle = throttle
            client.setCarControls(car_controls)
            car_controls.brake = 0  
            #time.sleep(10)
            throttle_values.append(throttle)
            for j in range(10):
                
                car_state = client.getCarState()
                x_acc =  car_state.kinematics_estimated.linear_acceleration.x_val
                y_acc =  car_state.kinematics_estimated.linear_acceleration.y_val
                z_acc =  car_state.kinematics_estimated.linear_acceleration.z_val
                acceleration_list[i,j,:] =  np.array([x_acc, y_acc, z_acc])

                x_vel = car_state.kinematics_estimated.linear_velocity.x_val
                y_vel = car_state.kinematics_estimated.linear_velocity.y_val
                z_vel = car_state.kinematics_estimated.linear_velocity.z_val
                velocity_list[i,j,:] =  np.array([x_vel, y_vel, z_vel])

                velocity = np.array([x_vel, y_vel, z_vel])

                print("acceleration")
                print(np.array([x_acc, y_acc, z_acc]))
                # print("velocity")

                # print(np.array([x_vel, y_vel, z_vel]))


                time.sleep(1)

            #print("car state: %s" % car_state)
        acceleration_norm = np.linalg.norm(acceleration_list, axis=2)
        acceleration_norm_avg = np.mean(acceleration_norm, axis = 1)
        plt.figure()
        plt.plot(throttle_values, acceleration_norm_avg, marker = 'o')
        plt.title("Acceleration vs Throttle")
        plt.xlabel("Throttle")
        plt.ylabel("Acceleration")
        plt.grid(True)
        plt.show()



    except KeyboardInterrupt:
        # Restore to original state
        client.reset()
        client.enableApiControl(False)


    # Restore to original state
    client.reset()
    client.enableApiControl(False)

    



