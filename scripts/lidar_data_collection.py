"""Collect LiDAR data

Settings: 'settings_lidar.json'

"""

import airsim_data_collection.common.setup_path
import airsim
import os
import numpy as np
import time
import shutil
import argparse

from airsim_data_collection.sensors.lidar_handler import LidarHandler


## -------------------------- PARAMETERS ------------------------ ##
# - - - - - - - - adjust these to match settings.json - - - - - - - - #
num_drones = 1                  # total number of drones in simulation
start_locs = np.array([[1,0]])  # initialization locations
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
#z = [0.9, 0.75]                          # flying altitudes
z = [5.0, 2.0, 20.0]
drone_names = ["Drone" + str(i) for i in range(num_drones)]

global_frame = False            # whether to shift drone coordinates to global frame

# MoveOnPath settings (https://microsoft.github.io/AirSim/apis/#apis-for-multirotor)
drivetrain = airsim.DrivetrainType.MaxDegreeOfFreedom  # Don't care where front points
                                                       # as opposed to "airsim.DrivetrainType.ForwardOnly"
#drivetrain = airsim.DrivetrainType.ForwardOnly
yaw_mode = airsim.YawMode(False, 0)  # (yaw_or_rate, is_rate)
                                    # 
speed = 4    # speed
lookahead = 15  # how far to look ahead, default = -1 (auto-decide)
adaptive_lookahead = 1  # whether to apply adaptive lookahead, default = 1 (yes)

# Data collection parameters
interval = 0.2             # time between collections in seconds
                           # data collection has 0.1 second overhead -> frequency = 1/(interval+0.1)
num_collections = 225
timeout = 600
env_name = 'blocks'  # user set this
run_name = '/' + env_name + '_loop_' + str(num_collections) + '_samples_' + str(int(1/(interval+0.1))) + 'hz_spd_' + str(speed) + "_noyaw"
folder_name = '/lidar_slam/blocks/'

paths = []

'''Blocks environment

   Travel across all the blocks
   
   Initial pose:
    "X": 25, "Y": -100, "Z": 0,
    "Pitch": 0, "Roll": 0, "Yaw": 180

'''
# paths.append([airsim.Vector3r(-25,20,-z[0]),
#               airsim.Vector3r(-25,100,-z[0]),
#               airsim.Vector3r(-25,180,-z[0]),
#               airsim.Vector3r(0,200,-z[0])])

'''Blocks environment

   Loop around single block
   
   Initial pose:
    "X": 0, "Y": 0, "Z": 0,
    "Pitch": 0, "Roll": 0, "Yaw": -90

'''
# paths.append([airsim.Vector3r(0, -60, -z[0]),
#               airsim.Vector3r(-50, -60, -z[0]),
#               airsim.Vector3r(-50, 0, -z[0]),
#               airsim.Vector3r(0, 0, -z[0]),
#               airsim.Vector3r(0, -60, -z[1]),
#               airsim.Vector3r(-50, -60, -z[1]),
#               airsim.Vector3r(-50, 0, -z[1]),
#               airsim.Vector3r(0, 0, -z[1]),
#               airsim.Vector3r(0, -60, -z[2]),
#               airsim.Vector3r(-50, -60, -z[2]),
#               airsim.Vector3r(-50, 0, -z[2]),
#               airsim.Vector3r(0, 0, -z[2])])

paths.append([airsim.Vector3r(0, -60, -z[0]),
              airsim.Vector3r(-50, -60, -z[0]),
              airsim.Vector3r(-50, 0, -z[0]),
              airsim.Vector3r(0, 0, -z[0])])


'''Blocks environment

   Cover "sub_1" area
   
   Initial pose:
    "X": 35, "Y": -90, "Z": 0,
    "Pitch": 0, "Roll": 0, "Yaw": 180

'''
# Goes behind big block at the top, only see one plane
# paths.append([airsim.Vector3r(-35, 0, -z[0]),
#               airsim.Vector3r(-35, 30, -z[0]),
#               airsim.Vector3r(-80, 30, -z[0]),
#               airsim.Vector3r(-80, 85, -z[0]),
#               airsim.Vector3r(-35, 85, -z[0]),
#               airsim.Vector3r(-35, 60, -z[0]),
#               airsim.Vector3r(35, 60, -z[0]),
#               airsim.Vector3r(35, 130, -z[0]),
#               airsim.Vector3r(-35, 130, -z[0]),  
#               airsim.Vector3r(-35, 95, -z[0]),
#               airsim.Vector3r(-80, 105, -z[0]),
#               airsim.Vector3r(-80, 160, -z[0]),
#               airsim.Vector3r(-35, 160, -z[0]),
#               airsim.Vector3r(-35, 195, -z[0]),
#               airsim.Vector3r(0, 195, -z[0])])

# paths.append([airsim.Vector3r(-35, 0, -z[0]),
#               airsim.Vector3r(-35, 85, -z[0]),
#               airsim.Vector3r(-80, 85, -z[1]),
#               airsim.Vector3r(-80, 30, -z[1]),
#               airsim.Vector3r(-35, 30, -z[1]),
#               airsim.Vector3r(-35, 150, -z[1]),
#               airsim.Vector3r(-80, 150, -z[1]),
#               airsim.Vector3r(-80, 95, -z[1]),
#               airsim.Vector3r(-35, 95, -z[0]),
#               airsim.Vector3r(-35, 180, -z[0]),
#               airsim.Vector3r(0, 180, -z[0])])

'''Building 99 environment

   Kitchen corridor

    Initial pose:
    "X": 0, "Y": 0, "Z": 0,
    "Pitch": 0, "Roll": 0, "Yaw": 0

'''
# paths.append([airsim.Vector3r(4, 0, -0.7),
#               airsim.Vector3r(4, -5, -0.7),
#               airsim.Vector3r(4, -10, -0.7),
#               airsim.Vector3r(4, -15, -0.6),
#               airsim.Vector3r(5, -20, -0.6),
#               airsim.Vector3r(5, -38, -0.7),
#               airsim.Vector3r(20, -38, -0.7)])

'''Building 99

    Unknown (possibly loops in central area)

'''
# paths.append([airsim.Vector3r(-3,0,-0.75),
#               airsim.Vector3r(-3,-2.5,-0.75),
#               airsim.Vector3r(-12,-2.5,-0.75),
#               airsim.Vector3r(-13,12,-0.75),
#               airsim.Vector3r(-17,33,-0.75)])

# paths.append([airsim.Vector3r(4,-22,-0.5),
#               airsim.Vector3r(6,-26,-0.75),
#               airsim.Vector3r(4,-38,-0.75),
#               airsim.Vector3r(22,-39,-0.75)])

# paths.append([airsim.Vector3r(-14,0,-z[0]),
#               airsim.Vector3r(-14,18,-z[0])])

# paths.append([airsim.Vector3r(-12,0,-z[1]),
#               airsim.Vector3r(-12,-8,-z[1])])

drone_pos = np.zeros((num_drones, 2, num_collections))

if __name__ == "__main__":

    # Connect to client
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("Arming drones...")
    for i in range(num_drones):
        client.enableApiControl(True, "Drone"+str(i))
        client.armDisarm(True, "Drone"+str(i))

    # Setup data collection
    print("Initializing data collection...")
    os.chdir('C:/Users/Adam/NAVLAB/AirSim/Data') # user dependent path
    datapath = os.getcwd()
    base_folder = datapath + folder_name + run_name
    pose_folder = base_folder + '/poses/'
    lidar_folder = base_folder + '/lidar/'
    drone_folders = ['/' + drone_names[i] + '/' for i in range(num_drones)]

    # Initialize data collection module for each drone
    sensors = num_drones * [None]
    for i in range(num_drones):
        sensors[i] = LidarHandler(client, lidar_folder=lidar_folder + drone_folders[i], 
                                  pose_folder=pose_folder + drone_folders[i])

    # Begin by creating new directory for data
    try:
        if os.path.isdir(base_folder):
            shutil.rmtree(base_folder)
        os.mkdir(base_folder)
        os.mkdir(lidar_folder)
        os.mkdir(pose_folder)
        for i in range(num_drones):
            os.mkdir(lidar_folder + drone_folders[i])
            os.mkdir(pose_folder + drone_folders[i])
    except OSError:
        print ("Creation of the directory failed")
    else:
        print ("Successfully created the directory")

    # Take off
    print("Taking off...")
    cmds = []
    for i in range(num_drones):
        cmd = client.takeoffAsync(vehicle_name=drone_names[i])
        cmds.append(cmd)

    for cmd in cmds:
        cmd.join()

    print("Ascend to hover altitude")
    cmds = []
    for i in range(num_drones):
        cmd = client.moveToZAsync(z=-z[i], velocity=1, vehicle_name=drone_names[i])
        cmds.append(cmd)
    for cmd in cmds:
        cmd.join()

    try:
        # Begin flying
        print("Flying routes")
        cmds = []
        for i in range(num_drones):
            path = paths[i]
            # Move command
            print(path)
            cmd = client.moveOnPathAsync(path, speed, timeout, drivetrain, 
                            yaw_mode, lookahead, adaptive_lookahead, vehicle_name=drone_names[i])
            cmds.append(cmd)

        # Data collection
        for i in range(num_collections):
            for j in range(num_drones):
                sensors[j].collect_data(drone_names[j])

                state = client.getMultirotorState(vehicle_name=drone_names[j])
                drone_pos[j,0,i] = state.kinematics_estimated.position.x_val + start_locs[j,0]
                drone_pos[j,1,i] = state.kinematics_estimated.position.y_val + start_locs[j,1]

            print('Collecting data ' + str(i))
            time.sleep(interval)
        
        for cmd in cmds:
            cmd.join()

    except KeyboardInterrupt:
        print("Disarming...")
        for i in range(num_drones):
            client.armDisarm(False, drone_names[i])

        print("Resetting")
        client.reset()

        for i in range(num_drones):
            client.enableApiControl(False, drone_names[i])
        print("Done.")


    print("Disarming...")
    for i in range(num_drones):
        client.armDisarm(False, drone_names[i])

    print("Resetting")
    client.reset()

    for i in range(num_drones):
        client.enableApiControl(False, drone_names[i])
    print("Done.")
        

    



