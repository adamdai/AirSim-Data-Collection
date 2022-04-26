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

from mySensorData import mySensorData

# Flies multiple drones through the streets of neighborhood
# assume all drones start in CC

parser = argparse.ArgumentParser()
parser.add_argument('--postprocess', action='store_true', default=False, 
        help="whether to only run postprocess step (in even previous run failed)")

args = parser.parse_args()

## -------------------------- PARAMETERS ------------------------ ##
# - - - - - - - - adjust these to match settings.json - - - - - - - - #
num_drones = 1                  # total number of drones in simulation
start_locs = np.array([[0,0], [3,0], [3,-3], [3,3], [0,-3], [0,3], [-3,-3], [-3,0], [-3,3]])  # initialization locations 
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
z = 1                          # flying altitude
z_range = [-3, -5]              # range of flight altitudes
global_frame = False            # whether to shift drone coordinates to global frame
path_length = 15
loops = 20
speed = 5
lookahead = 20
adaptive_lookahead = 1

# data collection parameters
interval = 1
num_collections = 20
timeout = num_collections
save_folder = '/' + str(num_collections) + '_samples_' + str(num_drones) + '_drones_street/'
val_split = 0.2


adj = {'NW': ['CW', 'NC'],
       'NC': ['NW', 'CC', 'NE'],
       'NE': ['NC', 'CE'],
       'CW': ['NW', 'CC', 'SW'],
       'CC': ['CW', 'NC', 'CE', 'SC'],
       'CE': ['CC', 'NE', 'SE'],
       'SW': ['CW', 'SC'],
       'SC': ['SW', 'CC', 'SE'],
       'SE': ['SC', 'CE']}

gridlen = 130

coords = {'NW': [gridlen, -gridlen],
          'NC': [gridlen, 0.0],
          'NE': [gridlen, gridlen],
          'CW': [0.0, -gridlen],
          'CC': [0.0, 0.0],
          'CE': [0.0, gridlen],
          'SW': [-gridlen, -gridlen],
          'SC': [-gridlen, 0.0],
          'SE': [-gridlen, gridlen]}

pathwidth = 0

preset = ['NC','NE','CE','CC',
          'CW','NW','NC','CC',
          'SC','SW','CW','CC',
          'CE','SE','SC','CC']

def gen_waypt(loc):
    waypt = coords[loc].copy()
    waypt[0] += random.uniform(-pathwidth, pathwidth)
    waypt[1] += random.uniform(-pathwidth, pathwidth)
    return waypt

# generate random route
def gen_path(drone):
    state = ['CC', '']
    path = []
    for i in range(path_length):
        # prevent backtracking
        cands = adj[state[0]]
        if state[1] in cands:
            cands.remove(state[1])
        # choose new segment randomly
        state[1] = state[0]
        state[0] = random.choice(cands)
        waypt = gen_waypt(state[0])
        path.append(airsim.Vector3r(waypt[0],waypt[1],z_list[drone]))
    return path

# fly preset route with random altitudes
def preset_path(loops, drone):
    path = []
    for i in range(loops):
        for j in range(len(preset)):
            waypt = gen_waypt(preset[j])
            if global_frame:
                waypt[0] -= start_locs[drone][0]
                waypt[1] -= start_locs[drone][1]
            #path.append(airsim.Vector3r(waypt[0],waypt[1],random.uniform(z_range[0], z_range[1])))
            path.append(airsim.Vector3r(waypt[0],waypt[1],z))
    return path

# postprocess poses to generate labels
def gen_labels(sensors, count, pose_folder):
    
    for i in range(count):
        fname = count*[None]
        fname[0] = 'Drone0' + ("pose%.6d.txt" % i)
        camname = 'Drone0' + ("campose%.6d.txt" % i)
        pose0 = sensors.read_pickle_file(pose_folder+fname[0])
        camera_info = sensors.read_pickle_file(pose_folder+camname)

        for j in range(num_drones-1):
            fname[j] = 'Drone' + str(j+1) + ("pose%.6d.txt" % i)

        for j in range(num_drones-1):
            posej = sensors.read_pickle_file(pose_folder+fname[j])
            sensors.saveLabel(start_locs, j+1, camera_info, posej, i, 'Car')


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
    os.chdir('C:/Users/Adam/Projects/NAVLab/AirSim/Data')
    datapath = os.getcwd()
    save_folder = datapath + save_folder
    base_folder = save_folder + '/training/'
    cam_folder = base_folder + '/image_2/'
    lidar_folder = base_folder + '/velodyne/'
    pose_folder = base_folder + '/pose/'
    calib_folder = base_folder + '/calib/'
    label_folder = base_folder + '/label_2/'
    split_folder = save_folder + '/ImageSets/'

    # initialize data collection module
    sensors = mySensorData(client, 
                        compress_img=False, cam_folder=cam_folder, lidar_folder=lidar_folder, 
                        pose_folder=pose_folder, calib_folder=calib_folder, label_folder=label_folder)

    # If not in postprocess mode, begin by creating new directory for data
    if not args.postprocess:
    
        try:
            if os.path.isdir(save_folder):
                shutil.rmtree(save_folder)
            os.mkdir(save_folder)
            os.mkdir(base_folder)
            os.mkdir(cam_folder)
            os.mkdir(lidar_folder)
            os.mkdir(pose_folder)
            os.mkdir(calib_folder)
            os.mkdir(label_folder)
            os.mkdir(split_folder)
        except OSError:
            print ("Creation of the directory failed")
        else:
            print ("Successfully created the directory")

        # initial scan
        sensors.collectData("Drone0", get_cam_data = True, get_lidar_data = False, get_calib_data=False,
                    cam_num=0, lidar_num=0, pose_num=0, calib_num=0)
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
                path = preset_path(loops,i)
                # move command
                print(path)
                cmd = client.moveOnPathAsync(path, speed, timeout, airsim.DrivetrainType.ForwardOnly, 
                                airsim.YawMode(False,0), lookahead, adaptive_lookahead, vehicle_name="Drone"+str(i))
                cmds.append(cmd)

            # data collection
            count = 0
            for i in range(1,num_collections):
                sensors.collectData("Drone0", get_cam_data = True, get_lidar_data = False, get_calib_data=False,
                            cam_num = i, lidar_num = i, pose_num = i, calib_num=i)
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

            gen_labels(sensors, count, pose_folder)
            print("label files created")

            gen_split(split_folder, count, val_split)
            print("split files created")
            raise


        print("disarming...")
        for i in range(num_drones):
            client.armDisarm(False, "Drone"+str(i))

        print("resetting")
        client.reset()

        for i in range(num_drones):
            client.enableApiControl(False, "Drone"+str(i))
        print("done.")

    # postprocessing
    gen_labels(sensors, count, pose_folder)
    print("label files created")

    



