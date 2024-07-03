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
    car_controls.brake = 0
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
            car_controls.brake = 0  

            client.setCarControls(car_controls)
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
            car_controls.throttle = 0
            car_controls.brake = 1            
            client.setCarControls(car_controls)

            time.sleep(4)

            #print("car state: %s" % car_state)
        print(acceleration_list)
        # Extract the x acceleration values (assuming x is the first element in the third dimension)
        x_accelerations = acceleration_list[:, :, 0]
        y_accelerations = acceleration_list[:, :, 1]
        z_accelerations = acceleration_list[:, :, 2]



# Calculate the average of the x accelerations for each group of 10
        average_x_accelerations = np.mean(x_accelerations, axis=1)
        average_y_accelerations = np.mean(y_accelerations, axis=1)
        average_z_accelerations = np.mean(z_accelerations, axis=1)

        test_average_x_accelerations = np.mean(x_accelerations, axis=0)
        test_average_y_accelerations = np.mean(y_accelerations, axis=0)
        test_average_z_accelerations = np.mean(z_accelerations, axis=0)




        
        acceleration_norm = np.linalg.norm(acceleration_list, axis=2)
        acceleration_norm_avg = np.mean(acceleration_norm, axis = 1)
        plt.figure()

        fig, axs = plt.subplots(2, 2)
        axs[0, 0].plot(throttle_values, average_x_accelerations)
        axs[0, 0].set_title("x acceleration")
        
        axs[1, 0].plot(throttle_values, average_y_accelerations)
        axs[1, 0].set_title("y acceleration")
        
        axs[1, 0].sharex(axs[0, 0])

        axs[0, 1].plot(throttle_values, average_z_accelerations)
        axs[0, 1].set_title("z acceleration")

        axs[1, 1].plot(throttle_values, acceleration_norm_avg, marker = 'o')
        axs[1, 1].set_title("Norm vs throttle")

        plt.grid(True)
        plt.show()

        
        plt.figure()
        #test other axis?
        fig, axs = plt.subplots(2, 2)
        axs[0, 0].plot(throttle_values, test_average_x_accelerations)
        axs[0, 0].set_title("t_x acceleration")
        
        axs[1, 0].plot(throttle_values, test_average_y_accelerations)
        axs[1, 0].set_title("t_y acceleration")
        
        axs[1, 0].sharex(axs[0, 0])

        axs[0, 1].plot(throttle_values, test_average_z_accelerations)
        axs[0, 1].set_title("t_z acceleration")

        axs[1, 1].plot(throttle_values, acceleration_norm_avg, marker = 'o')
        axs[1, 1].set_title("t_Norm vs throttle")

        plt.grid(True)
        plt.show()


    except KeyboardInterrupt:
        # Restore to original state
        client.reset()
        client.enableApiControl(False)


    # Restore to original state
    client.reset()
    client.enableApiControl(False)

    



