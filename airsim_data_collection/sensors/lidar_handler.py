import airsim_data_collection.common.setup_path 
import airsim

import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import time
import pickle


class LidarHandler:
    """LiDAR Data Collection class

    Use to collect LiDAR point clouds and pose data

    Attributes
    ----------
    client : AirSim VehicleClient
        AirSim client object
    car : bool
        True for car, False for multirotor
    lidar_folder : str
        Folder to store LiDAR point cloud data
    pose_folder : str
        Folder to store LiDAR pose data
    start_time : float
        Time of intialization
    frame_num : int
        Internal frame counter for data collections

    Methods
    -------

    """
    def __init__(self, client, car=False, lidar_folder=None, pose_folder=None):
        self.client = client
        self.car = car
        self.lidar_folder = lidar_folder 
        self.pose_folder = pose_folder 

        self.start_time = time.time()
        self.frame_num = 1

        self.prev_time = time.time()


    def collect_data(self, vehicle_name):
        """Collect LiDAR data from drone

        Parameters
        ----------
        vehicle_name : str
            Vehicle name 

        """
        self.save_lidar(vehicle_name)
    

    def save_lidar(self, vehicle_name):
        """Save LiDAR data to lidar_folder

        Parameters
        ----------
        vehicle_name : str
            Vehicle name 
        
        Generates
        ---------
        .bin file
            File containing LiDAR point cloud

        """
        #start_time = time.time()
        points, lidar_pose, time_stamp = self.get_lidar(vehicle_name)
        #print("lidar timestamp ", self.frame_num, ": ", time_stamp/1e9 - self.start_time)
        #print("lidar collection time: ", time.time() - start_time)
        print("lidar collection time: ", time.time() - self.prev_time)
        self.prev_time = time.time()

        # Convert lidar pose from vehicle frame to world frame
        # TODO

        # Save LiDAR pose
        pos = lidar_pose.position
        ori = lidar_pose.orientation
        filename_pose = self.pose_folder + ("%.6d.txt" % self.frame_num)
        op = open(os.path.normpath(filename_pose), 'a+')
        op.write(str(pos.x_val) + ', ' + str(pos.y_val) + ', ' + str(pos.z_val) + '\n')
        op.write(str(ori.x_val) + ', ' + str(ori.y_val) + ', ' + str(ori.z_val) + ', ' + str(ori.w_val))
        op.close()

        # Save LiDAR points in velodyne folder as bin 
        filename_pc = self.lidar_folder + ("%.6d.bin" % self.frame_num)
        n_points = points.shape[0]
        # Append column of 1.0s for intensity
        final_points = np.hstack((points, np.ones((n_points,1), dtype=np.dtype('f4'))))
        final_points.tofile(filename_pc)
        
        self.frame_num += 1


    def get_lidar(self, vehicle_name): 
        """Get LiDAR data

        Note: get_lidar takes about 0.1 seconds to run

        Parameters
        ----------
        vehicle_name : str
            Vehicle name to query for LiDAR data
        
        Returns
        -------
        points_all : np.array (n_pts x 3)
            Array of LiDAR points
        lidar_pose: AirSim Pose
            LiDAR pose
        time_stamp : TTimePoint (nanoseconds)
            Timestamp of data

        """
        lidar_data = self.client.getLidarData(vehicle_name=vehicle_name)
        time_stamp = lidar_data.time_stamp
        lidar_pose = lidar_data.pose

        points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0]/3), 3))
        print('lidar points: ' + str(points.shape))
        
        return points, lidar_pose, time_stamp


    






