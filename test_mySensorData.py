import setup_path 
import airsim

import os
from mySensorData import mySensorData
import numpy as np

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
#client.reset()
client.armDisarm(True)
airsim.wait_key('Press any key to take an image and point cloud data')


os.chdir('C:/Users/Adam/Projects/NAVLab/AirSim/Data')
datapath = os.getcwd()
base_folder1 = '/label_testing/'
base_folder = base_folder1 + '/training/'
cam_folder = datapath + base_folder + '/image_2/'
lidar_folder = datapath + base_folder + '/velodyne/'
pose_folder = datapath + base_folder + '/pose/'
calib_folder = datapath + base_folder + '/calib/'
label_folder = datapath + base_folder + '/label_2/'
try:
    os.mkdir(datapath + base_folder1)
    os.mkdir(datapath + base_folder)
    os.mkdir(cam_folder)
    os.mkdir(lidar_folder)
    os.mkdir(pose_folder)
    os.mkdir(calib_folder)
    os.mkdir(label_folder)
except OSError:
    print ("Creation of the directory failed")
else:
    print ("Successfully created the directory")
sensors = mySensorData(client, 
                 compress_img = False, cam_folder = cam_folder, lidar_folder = lidar_folder, 
                 pose_folder = pose_folder, calib_folder = calib_folder, label_folder=label_folder)

airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
# client.takeoffAsync().join()
# client.moveToPositionAsync(-10, 10, -10, 5).join()
airsim.wait_key('Press any key to collect data')
sensors.collectData(vehicle_name = 'Drone0', get_cam_data = True, get_lidar_data = False, get_calib_data = True)
sensors.collectData(vehicle_name = 'Drone1')

airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)

num_files = 1 # number of time steps
N = 1 # 1 other drone
start_locs = np.array([[0,0], [5,0]])
# cam_matrix = make_transform(np.array([[0,0,1],[0,-1,0],[1,0,0]]),np.array([0.5,0,0.1]))
# os.chdir('/home/navlab-admin/AirSim-MOT/data')
# datapath = os.getcwd()
# foldername = '/test'
# datafolder = datapath + foldername + '/pose/'
# savefolder = datapath + foldername + '/processed/label/'
#####
# start0 = start_locs[0,:]
# transform_matrix_list = np.zeros((N,4,4))

# for i in range(N):
#     starti = start_locs[i+1,:]
#     ti = np.append((starti - start0),0)
#     transformi = make_transform(np.eye(3),ti)
#     transform_matrix_list[i,:,:] = transformi

for i in range(num_files):
    fname = num_files*[None]
    fname[0] = 'Drone0' + ("pose%.6d.txt" % i)
    camname = 'Drone0' + ("campose%.6d.txt" % i)
    pose0 = sensors.read_pickle_file(pose_folder+fname[0])
    camera_info = sensors.read_pickle_file(pose_folder+camname)

    for j in range(N):
        fname[j] = 'Drone' + str(j+1) + ("pose%.6d.txt" % i)

    for j in range(N):
        posej = sensors.read_pickle_file(pose_folder+fname[j])
        sensors.saveLabel(start_locs, j+1, camera_info, posej, i, 'Car')
        # dronej = open(pose_folder+fname[j], 'rb')
        # posj = pickle.load(dronej)
        # transform_init = transform_matrix_list[j,:,:]
        # dataj = get_items_of_a_label_row(pos0, posj, transform_init, cam_matrix)
        # write_to_label_file(dataj,'Car',savefolder+("%.6d.txt" % i))
        # dronej.close()
