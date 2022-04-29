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


parser = argparse.ArgumentParser()
parser.add_argument('--postprocess', action='store_true', default=False, 
        help="whether to only run postprocess step (in event that previous run failed)")

args = parser.parse_args()

## -------------------------- PARAMETERS ------------------------ ##
# - - - - - - - - Adjust these to match settings.json - - - - - - - - #
start_locs = np.array([[10,0], [0,0]])  # x-y initialization coordinates
z = [30, 70, 20, 20]
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
global_frame = False            # Whether to shift drone coordinates to global frame
speed = 5
lookahead = 20
adaptive_lookahead = 1

# Data collection parameters
interval = 0.1              # Time between collections in seconds
num_collections = 50
timeout = 120
env_name = 'blocks'
run_name = '/' + env_name + '_' + str(num_collections) + '_samples'

# Heading controller 
kp_steer = 0.5

# Throttle controller
follow_dist = 10.0 
kp_throttle = 0.1

# Vehicle control limits
min_steer = -0.5
max_steer = 0.5
min_throttle = 0.75
max_throttle = 1.5

# Leader trajectory


## -------------------------- METHODS ------------------------ ##


## -------------------------- MAIN ------------------------ ##
if __name__ == "__main__":

    # Connect to client
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True, "Car1")
    client.enableApiControl(True, "Car2")

    car1_controls = airsim.CarControls()
    car2_controls = airsim.CarControls()

    # Setup data collection
    print("Initializing data collection...")
    os.chdir('C:/Users/Adam/NAVLAB/AirSim/Data')  # User dependent path
    datapath = os.getcwd()
    base_folder = datapath + '/car_leader_follower/' + run_name
    cam_folder = base_folder + '/images/'
    pose_folder = base_folder + '/poses/'

    # Initialize data collection module
    sensors = SensorHandler(client, car=True, compress_img=False, cam_folder=cam_folder, pose_folder=pose_folder)
    
    # Create new directory for data
    try:
        if os.path.isdir(base_folder):
            shutil.rmtree(base_folder)
        os.mkdir(base_folder)
        os.mkdir(cam_folder)
        os.mkdir(pose_folder)
    except OSError:
        print ("Creation of the directory failed")
    else:
        print ("Successfully created the directory")

    # Begin driving and data collection
    try:
        print("driving routes")
        count = 0
        for i in range(num_collections):

            # Car 2 preset trajectory
            car2_controls.steering = 0.0  #0.2 * np.sin(0.01 * count)
            car2_controls.throttle = 1.0
            client.setCarControls(car2_controls, "Car2")

            # Car 1 follow control
            # - Get vehicles positions and headings
            car1_state = client.getCarState("Car1")
            car1_pos = airsimpos2np(car1_state.kinematics_estimated.position)
            car1_pos[0:2] += start_locs[0]  # Adjust for initialization offset (shift to global coords)
            car1_ori = car1_state.kinematics_estimated.orientation
            _,_,car1_heading = quaternion_to_eul(car1_ori)

            car2_state = client.getCarState("Car2")
            car2_pos = airsimpos2np(car2_state.kinematics_estimated.position)
            car2_pos[0:2] += start_locs[1]  # Adjust for initialization offset (shift to global coords)
            car2_ori = car2_state.kinematics_estimated.orientation
            _,_,car2_heading = quaternion_to_eul(car2_ori)

            # - Compute goal pose from car 2 pose
            theta = car2_heading - np.pi  # Angle pointing behind car 2
            dx = follow_dist * np.cos(theta)  # x-displacement of goal from car 2
            dy = follow_dist * np.sin(theta)  # y-displacement of goal from car 2
            goal_pos = car2_pos + np.array([dx,dy,0])

            # - Heading control
            car1_to_goal = np.arctan2(goal_pos[1] - car1_pos[1], goal_pos[0] - car1_pos[0])  # Angle between car1_pos and goal_pos
            heading_err = angle_diff(car1_heading, car1_to_goal)
            car1_controls.steering = np.clip(kp_steer * heading_err, min_steer, max_steer)
            #print('heading_err: ' + str(heading_err))
            print('car 1 to goal angle: ' + str(car1_to_goal))
            print('car 1 heading: ' + str(car1_heading))

            # - Throttle control 
            dist_err = np.linalg.norm(car1_pos - goal_pos)
            car1_controls.throttle = np.clip(1.0 + kp_throttle * dist_err, min_throttle, max_throttle)
            client.setCarControls(car1_controls, "Car1")
            #print('throttle: ' + str(car1_controls.throttle))

            # Data collection
            sensors.collect_data("Car1", get_cam_data = True, get_lidar_data = False, get_calib_data=False,
                        cam_num = i, pose_num = i)
            sensors.collect_data("Car2", get_cam_data = False, get_lidar_data = False, get_calib_data=False,
                        cam_num = i, pose_num = i)
            #print('collecting data ' + str(i))
            time.sleep(interval)
            count += 1

    except KeyboardInterrupt:
        # Restore to original state
        client.reset()
        client.enableApiControl(False)


    # Restore to original state
    client.reset()
    client.enableApiControl(False)

    



