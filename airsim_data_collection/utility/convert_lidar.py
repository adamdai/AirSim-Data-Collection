"""Script to convert LiDAR data from (??) to .bin KITTI format.

"""

import numpy as np
import os
import pickle
import argparse


# Input data dir
os.chdir('/home/navlab-admin/AirSim-MOT/data/')
datapath = os.getcwd()
foldername = 'test'
datafolder = datapath + foldername + '/lidar/'

# Output data dir
outpath = datapath + foldername + '/processed/velodyne/'

def convert_lidar(foldername):
    min_points = 1e10

    datafolder = datapath + foldername + '/lidar/'
    outpath = datapath + foldername + '/processed/velodyne/'

    for file in os.listdir(datafolder):
        if file.startswith('scan'):
            lidarfname = open(datafolder+file, 'rb')
            id = file[4:10]
            points = pickle.load(lidarfname)
            n_points = points.shape[0]
            if n_points < min_points:
                min_points = n_points
            final_points = np.hstack((points, np.ones((n_points,1), dtype=np.dtype('f4'))))
            final_points.tofile(outpath+id+'.bin')

    print(min_points)

if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser(description="Convert lidar data")
    arg_parser.add_argument("foldername", type=str, help="path to data")   
    args = arg_parser.parse_args() 

    foldername = args.foldername

    convert_lidar(foldername)