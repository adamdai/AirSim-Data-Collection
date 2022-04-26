import setup_path
import airsim
import pprint
import math
import random
import os
import numpy as np
import matplotlib.pyplot as plt

import sys
import time
import shutil
import argparse

from mySensorData import mySensorData

# Collect data for lidar slam

parser = argparse.ArgumentParser()
parser.add_argument('--postprocess', action='store_true', default=False, 
        help="whether to only run postprocess step (in even previous run failed)")

args = parser.parse_args()

## -------------------------- PARAMETERS ------------------------ ##
# - - - - - - - - adjust these to match settings.json - - - - - - - - #
num_drones = 1                  # total number of drones in simulation
start_locs = np.array([[1,0]])  # initialization locations
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
#z = [0.9, 0.75]                          # flying altitudes
z = [0.75, 0.75]
drone_names = ["Drone" + str(i) for i in range(num_drones)]

global_frame = False            # whether to shift drone coordinates to global frame
speed = 5
lookahead = -1
adaptive_lookahead = 0

# data collection parameters
interval = 1              # time between collections in seconds
num_collections = 10
timeout = 10
env_name = 'blocks'  # user set this
run_name = '/' + env_name + '_' + str(num_collections) + '_samples_1'

paths = []

# blocks environment
# paths.append([airsim.Vector3r(-25,20,-z[0]),
#               airsim.Vector3r(-25,100,-z[0]),
#               airsim.Vector3r(-25,180,-z[0]),
#               airsim.Vector3r(0,200,-z[0])])

paths.append([airsim.Vector3r(0,20,-z[0])])

# paths.append([airsim.Vector3r(-12,0,-z[1]),
#               airsim.Vector3r(-12,-8,-z[1]),
#               airsim.Vector3r(0,-8,-z[1]),
#               airsim.Vector3r(0,0,-z[1])])

# kitchen corridor
# paths.append([airsim.Vector3r(-3,0,-0.75),
#               airsim.Vector3r(-3,-2.5,-0.75),
#               airsim.Vector3r(-12,-2.5,-0.75),
#               airsim.Vector3r(-13,12,-0.75),
#               airsim.Vector3r(-17,33,-0.75)])

# paths.append([airsim.Vector3r(4,-22,-0.75),
#               airsim.Vector3r(6,-26,-0.75),
#               airsim.Vector3r(4,-38,-0.75),
#               airsim.Vector3r(22,-39,-0.75)])

# paths.append([airsim.Vector3r(-14,0,-z[0]),
#               airsim.Vector3r(-14,18,-z[0])])

# paths.append([airsim.Vector3r(-12,0,-z[1]),
#               airsim.Vector3r(-12,-8,-z[1])])

drone_pos = np.zeros((num_drones, 2, num_collections))

if __name__ == "__main__":

    # connect to client
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("arming drones...")
    for i in range(num_drones):
        client.enableApiControl(True, "Drone"+str(i))
        client.armDisarm(True, "Drone"+str(i))

    # setup data collection
    print("initializing data collection...")
    os.chdir('C:/Users/Adam/NAVLAB/AirSim/Data') # user dependent path
    datapath = os.getcwd()
    base_folder = datapath + '/lidar_slam/' + run_name
    pose_folder = base_folder + '/poses/'
    lidar_folder = base_folder + '/lidar/'
    drone_folders = ['/' + drone_names[i] + '/' for i in range(num_drones)]

    # initialize data collection module for each drone
    sensors = num_drones * [None]
    for i in range(num_drones):
        sensors[i] = mySensorData(client, compress_img=False, 
                        lidar_folder=lidar_folder + drone_folders[i], 
                        pose_folder=pose_folder + drone_folders[i])

    # If not in postprocess mode, begin by creating new directory for data
    if not args.postprocess:
    
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

        # initial scan
        # for i in range(num_drones):
        #     sensors.collectData(drone_names[i], get_cam_data = True, get_lidar_data = True,
        #             cam_num=0, lidar_num=0, pose_num=0)
        # print('collecting data ' + str(0))

        # take off
        print("taking off...")
        cmds = []
        for i in range(num_drones):
            cmd = client.takeoffAsync(vehicle_name=drone_names[i])
            cmds.append(cmd)
        for cmd in cmds:
            cmd.join()

        # print("ascend to hover altitude")
        # cmds = []
        # for i in range(num_drones):
        #     cmd = client.moveToZAsync(z=-z[i], velocity=1, vehicle_name=drone_names[i])
        #     cmds.append(cmd)
        # for cmd in cmds:
        #     cmd.join()

        try:
            # begin flying
            print("flying routes")
            cmds = []
            for i in range(num_drones):
                path = paths[i]
                # move command
                print(path)
                cmd = client.moveOnPathAsync(path, speed, timeout, airsim.DrivetrainType.MaxDegreeOfFreedom, 
                                airsim.YawMode(False,0), lookahead, adaptive_lookahead, vehicle_name=drone_names[i])
                cmds.append(cmd)

            # data collection
            count = 0
            for i in range(1,num_collections):
                for j in range(num_drones):
                    sensors[j].collectData(drone_names[j], get_lidar_data = True,
                            cam_num = i, lidar_num = i, pose_num = i)

                    state = client.getMultirotorState(vehicle_name=drone_names[j])
                    drone_pos[j,0,i] = state.kinematics_estimated.position.x_val + start_locs[j,0]
                    drone_pos[j,1,i] = state.kinematics_estimated.position.y_val + start_locs[j,1]

                print('collecting data ' + str(i))
                time.sleep(interval)
                count += 1
            
            for cmd in cmds:
                cmd.join()

        except KeyboardInterrupt:
            print("disarming...")
            for i in range(num_drones):
                client.armDisarm(False, drone_names[i])

            print("resetting")
            client.reset()

            for i in range(num_drones):
                client.enableApiControl(False, drone_names[i])
            print("done.")


        print("disarming...")
        for i in range(num_drones):
            client.armDisarm(False, drone_names[i])

        print("resetting")
        client.reset()

        for i in range(num_drones):
            client.enableApiControl(False, drone_names[i])
        print("done.")
        
        for i in range(num_drones):
            plt.plot(drone_pos[i,0,:], drone_pos[i,1,:])
        plt.show()

    



