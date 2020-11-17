import setup_path
import airsim
import pprint
import math
import random
import os
import numpy as np

import sys
import time
import shutil
import argparse
import itertools

from mySensorData import mySensorData

# Flies multiple drones through the streets of neighborhood
# assume all drones start in CC

parser = argparse.ArgumentParser()
parser.add_argument('--postprocess', action='store_true', default=False, 
        help="whether to only run postprocess step (in event that previous run failed)")

args = parser.parse_args()

## -------------------------- PARAMETERS ------------------------ ##
# - - - - - - - - adjust these to match settings.json - - - - - - - - #
num_drones = 3                 # total number of drones in simulation
start_locs = np.array([[4,0], [8,2], [6,-2]])  # initialization locations 
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
global_frame = False            # whether to shift drone coordinates to global frame
speed = 5
lookahead = 20
adaptive_lookahead = 1

# data collection parameters
interval = 1              # time between collections in seconds
num_collections = 120
timeout = num_collections * interval
run_name = '/' + str(num_collections) + '_samples_' + str(num_drones) + '_drones/'

paths = []

paths.append([airsim.Vector3r(125,0,-7),
              airsim.Vector3r(125,-130,-7),
              airsim.Vector3r(0,-130,-7),
              airsim.Vector3r(0,0,-7)])

paths.append([airsim.Vector3r(125,0,-7),
              airsim.Vector3r(125,-130,-7),
              airsim.Vector3r(0,-130,-7),
              airsim.Vector3r(0,0,-7)])

paths.append([airsim.Vector3r(125,0,-7),
              airsim.Vector3r(125,-130,-7),
              airsim.Vector3r(0,-130,-7),
              airsim.Vector3r(0,0,-7)])

# postprocess poses to generate labels
def gen_ranges(sensors, num_drones, count, pose_folder):

    for i in range(count):
        fname = num_drones*[None]

        for j in range(num_drones):
            fname[j] = 'Drone' + str(j) + ("pose%.6d.txt" % i)

        for a,b in itertools.combinations(range(num_drones), 2):
            pos_a = sensors.read_pickle_file(pose_folder+fname[a])
            pos_b = sensors.read_pickle_file(pose_folder+fname[b])
            sensors.saveRange(start_locs, a, b, pos_a, pos_b, i)


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
    os.chdir('C:/Users/Adam/Projects/NAVLab/AirSim/Data') # user dependent path
    datapath = os.getcwd()
    base_folder = datapath + '/inter_ranging/' + run_name
    cam_folder = base_folder + '/images/'
    pose_folder = base_folder + '/poses/'
    range_folder = base_folder + '/ranges/'

    # initialize data collection module
    sensors = mySensorData(client, 
                        compress_img=False, cam_folder=cam_folder, 
                        pose_folder=pose_folder, range_folder=range_folder)

    # If not in postprocess mode, begin by creating new directory for data
    if not args.postprocess:
    
        try:
            if os.path.isdir(base_folder):
                shutil.rmtree(base_folder)
            os.mkdir(base_folder)
            os.mkdir(cam_folder)
            os.mkdir(pose_folder)
            os.mkdir(range_folder)
        except OSError:
            print ("Creation of the directory failed")
        else:
            print ("Successfully created the directory")

        # initial scan
        sensors.collectData("Drone0", get_cam_data = True, get_lidar_data = False, get_calib_data=False,
                    cam_num=0, pose_num=0)
        for j in range(num_drones-1):
            sensors.collectData("Drone"+str(j+1), get_cam_data=False, get_lidar_data=False, pose_num=0)
        print('collecting data ' + str(0))

        # take off
        print("taking off...")
        cmds = []
        for i in range(num_drones):
            cmd = client.takeoffAsync(vehicle_name="Drone"+str(i))
            cmds.append(cmd)
        for cmd in cmds:
            cmd.join()

        try:
            # begin flying
            print("flying routes")
            cmds = []
            for i in range(num_drones):
                path = paths[i]
                # move command
                print(path)
                cmd = client.moveOnPathAsync(path, speed, timeout, airsim.DrivetrainType.ForwardOnly, 
                                airsim.YawMode(False,0), lookahead, adaptive_lookahead, vehicle_name="Drone"+str(i))
                cmds.append(cmd)

            # data collection
            count = 0
            for i in range(1,num_collections):
                sensors.collectData("Drone0", get_cam_data = True, get_lidar_data = False, get_calib_data=False,
                            cam_num = i, pose_num = i)
                for j in range(num_drones-1):
                    sensors.collectData("Drone"+str(j+1), get_cam_data = False, get_lidar_data = False, pose_num = i)

                print('collecting data ' + str(i))
                time.sleep(interval)
                count += 1
            
            for cmd in cmds:
                cmd.join()

        except KeyboardInterrupt:
            print("disarming...")
            for i in range(num_drones):
                client.armDisarm(False, "Drone"+str(i))

            print("resetting")
            client.reset()

            for i in range(num_drones):
                client.enableApiControl(False, "Drone"+str(i))
            print("done.")

            gen_ranges(sensors, num_drones, count, pose_folder)
            print("range files created")


        print("disarming...")
        for i in range(num_drones):
            client.armDisarm(False, "Drone"+str(i))

        print("resetting")
        client.reset()

        for i in range(num_drones):
            client.enableApiControl(False, "Drone"+str(i))
        print("done.")

    # postprocessing
    gen_ranges(sensors, num_drones, count, pose_folder)
    print("range files created")

    



