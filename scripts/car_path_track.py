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



## -------------------------- MAIN ------------------------ ##
if __name__ == "__main__":

    # Connect to client
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True)
    car_controls = airsim.CarControls()

    start_unreal = np.array([39835.236, 62827.409, 15630.430])
    goal_unreal = np.array([44570.0, -17780.0, 30930.0])
    goal_airsim = (goal_unreal - start_unreal)[:2] / 100.0

    goal = np.array([10.0, 10.0])

    # png_image = client.simGetImage("BirdsEyeCamera", airsim.ImageType.Scene)
    # print("captured image")
    # # save image
    # airsim.write_file(os.path.normpath('C:/Users/Adam/Documents/AirSim/image.png'), png_image)

    # get vehicle position
    car_state = client.getCarState()
    print("car state: %s" % car_state)


    try:
        print("driving routes")
        while(True):
            car_controls.steering = 0.0  #0.2 * np.sin(0.01 * count)
            car_controls.throttle = 1.0
            client.setCarControls(car_controls)
            time.sleep(1)

            car_state = client.getCarState()
            print("car state: %s" % car_state)


    except KeyboardInterrupt:
        # Restore to original state
        client.reset()
        client.enableApiControl(False)


    # Restore to original state
    client.reset()
    client.enableApiControl(False)

    



