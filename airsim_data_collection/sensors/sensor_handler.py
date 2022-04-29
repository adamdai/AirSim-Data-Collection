import airsim_data_collection.common.setup_path 
import airsim

import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import pprint
import cv2
import time
import pickle

from airsim_data_collection.utility.convert_calib import write_to_label_file
from airsim_data_collection.utility.convert_pose import get_transform, quaternion_to_eul, eul_to_rotmat


class SensorHandler:
    """Sensor Data Collection class

    Attributes
    ----------
    client : AirSim VehicleClient
        AirSim client object
    car : bool
        True for car, False for multirotor
    compress_img : bool
        Flag to compress images
    cam_folder : str
        Folder to store camera data
    lidar_folder : str
        Folder to store LiDAR data
    pose_folder : str
        Folder to store pose data
    calib_folder : str
        Folder to store calibration data

    Methods
    -------

    """
    def __init__(self, client, car=False, compress_img=False, cam_folder=None, 
                 lidar_folder=None, pose_folder=None, calib_folder=None, 
                 label_folder=None, range_folder=None):
        self.client = client
        self.car = car
        self.cam_folder = cam_folder  
        self.lidar_folder = lidar_folder 
        self.pose_folder = pose_folder 
        self.calib_folder = calib_folder 
        self.compress_img = compress_img
        self.label_folder = label_folder
        self.range_folder = range_folder

        self.start_time = time.time()

        # Transformation from AirSim NED camera to conventional camera 
        self.ned2cam = np.array([[0,1,0,0],[0,0,1,0],[1,0,0,0],[0,0,0,1]])


    def collect_data(self, vehicle_name, 
			        get_cam_data=False, get_lidar_data=False, get_calib_data=False,
			        cam_num=0, lidar_num=0, pose_num=0, calib_num=0):
        """Collect sensor data from drone

        Parameters
        ----------
        vehicle_name : str
            Vehicle name 
        get_cam_data : bool

        """
        lidar_pose = [None]
        camera_info = [None]

        self.save_pose(pose_num, vehicle_name)

        if get_cam_data:
            camera_info = self.save_img(cam_num, vehicle_name)

        if get_lidar_data:
            lidar_pose, lidar_points = self.save_lidar(lidar_num, vehicle_name)

        if get_calib_data:
            self.gen_calib(calib_num, vehicle_name, lidar_pose, camera_info)


    def save_img(self, im_num, vehicle_name): 
        """Save vehicle image to cam_folder

        Parameters
        ----------
        im_num : int
            Frame number
        vehicle_name : str
            Vehicle name 
        
        Generates
        ---------
        .png file
            Vehicle image

        """
        responses = self.get_img(vehicle_name)
        filename_img = self.cam_folder + ("%.6d.png" % im_num)
        filename_ts = self.pose_folder + ("cam_ts%.6d.txt" % im_num)

        for idx, response in enumerate(responses):
            if self.compress_img:  # png format
                airsim.write_file(os.path.normpath(filename_img), response.image_data_uint8)
            else:  # Uncompressed array
                img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
                img_rgb = img1d.reshape(response.height, response.width, 3)
                cv2.imwrite(os.path.normpath(filename_img), img_rgb)
            output = open(os.path.normpath(filename_ts), 'wb')
            pickle.dump(response.time_stamp, output)
            output.close()

        camera_info = self.client.simGetCameraInfo("0", vehicle_name=vehicle_name)
        filename_cam_pose = self.pose_folder + vehicle_name + ("campose%.6d.txt" % im_num)
        op1 = open(os.path.normpath(filename_cam_pose), 'wb')
        pickle.dump(camera_info,op1)
        op1.close()

        return camera_info
    

    def save_lidar(self, im_num, vehicle_name):
        """Save LiDAR data to lidar_folder

        Parameters
        ----------
        im_num : int
            Frame number
        vehicle_name : str
            Vehicle name 
        
        Generates
        ---------
        .bin file
            File containing LiDAR point cloud

        """
        points, lidar_pose, time_stamp = self.get_lidar(vehicle_name)
        print("lidar timestamp ", im_num, ": ", time_stamp/1e9 - self.start_time)

        # Save LiDAR pose

        # Save LiDAR points in velodyne folder as bin 
        filename_pc = self.lidar_folder + ("%.6d.bin" % im_num)
        n_points = points.shape[0]
        # Append column of 1.0s for intensity
        final_points = np.hstack((points, np.ones((n_points,1), dtype=np.dtype('f4'))))
        final_points.tofile(filename_pc)
        return lidar_pose, final_points
    

    def save_pose(self, im_num, vehicle_name):
        """Save vehicle pose data

        Parameters
        ----------
        im_num : int
            Frame number
        vehicle_name : str
            Vehicle name to query for pose
        
        Generates
        ---------
        .txt file
            File containing pose as (xyz) position and (xyzw) quaternion orientation

        """
        if self.car:
            state = self.client.getCarState(vehicle_name=vehicle_name)
        else:
            state = self.client.getMultirotorState(vehicle_name=vehicle_name)
        #pprint.pprint(vehicle_name + " position: " + str(state.kinematics_estimated.position))
        print("pose timestamp ", im_num, ": ", state.timestamp/1e9 - self.start_time)

        pos = state.kinematics_estimated.position
        ori = state.kinematics_estimated.orientation

        # Save normal .txt file with position and orientation
        filename_pose = self.pose_folder + ("%.6d.txt" % im_num)
        op = open(os.path.normpath(filename_pose), 'a+')
        op.write(str(pos.x_val) + ', ' + str(pos.y_val) + ', ' + str(pos.z_val) + '\n')
        op.write(str(ori.x_val) + ', ' + str(ori.y_val) + ', ' + str(ori.z_val) + ', ' + str(ori.w_val))
        op.close()


    def get_img(self, vehicle_name): 
        """Get Image

        Parameters
        ----------
        vehicle_name : str
            Vehicle name to query for image
        
        Returns
        -------
        responses : list[ImageResponse]
            list of ImageResponses

        """
        if self.compress_img:
            responses = self.client.simGetImages([
                            airsim.ImageRequest("0", airsim.ImageType.Scene)], vehicle_name=vehicle_name) 
        else:
            responses = self.client.simGetImages([
                            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name=vehicle_name)
        return responses


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
        # k = 1
        # points_list = []
        # for i in range(k):
        #     lidar_data = self.client.getLidarData(vehicle_name=vehicle_name)
        #     time_stamp = lidar_data.time_stamp
        #     lidar_pose = lidar_data.pose

        #     if (len(lidar_data.point_cloud) >= 3):
        #         points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
        #         points = np.reshape(points, (int(points.shape[0]/3), 3))
        #         points_list.append(points)
        #     time.sleep(0.001)
        #
        # if k == 1:
        #     points_all = points_list[0]
        # else:
        #     points_all = np.vstack((points_list[0],points_list[1]))
        #     for i in range(2,k):
        #         points_all = np.vstack((points_all,points_list[i]))
        
        lidar_data = self.client.getLidarData(vehicle_name=vehicle_name)
        time_stamp = lidar_data.time_stamp
        lidar_pose = lidar_data.pose

        points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0]/3), 3))
        pprint.pprint('lidar points: ' + str(points.shape))
        
        return points, lidar_pose, time_stamp


    def gen_calib(self, calib_num, vehicle_name, lidar_pose, camera_info):
        """Generate KITTI calib calibration file

        Parameters
        ----------
        calib_num : int
            Calibration frame number
        lidar_pose : AirSim Pose (position, orientation)
            LiDAR pose
        camera_info : AirSim CameraInfo (pose, FOV, projection matrix)
            Camera info struct

        Generates
        ---------
        .txt file
            File containing calibration and transformation matrices

        """
        lt = get_transform(lidar_pose)
        ct = get_transform(camera_info.pose, True)

        # velodyne to camera
        v2c = np.dot(self.ned2cam, np.dot(ct, lt))[0:3,:]
        # IMU to velodyne
        i2v = np.linalg.inv(lt)[0:3,:] # this is wrong
        R0 = np.eye(3)
        # camera projection matrix (wrong but not used by PointRCNN)
        P2 = np.asarray(camera_info.proj_mat.matrix)[0:3,:]
        # set camera center to image center (W/2, H/2)
        P2[0,3] = 960
        P2[1,3] = 540
        P2[1,2] = np.abs(P2[1,2])
        P0 = P2
        P1 = P2
        P3 = P2
        write_to_label_file(P0, P1, P2, P3, R0, v2c, i2v, self.calib_folder+("%.6d.txt" % calib_num))


    def save_label(self, start_locs, id, camera_info, pos2, label_num, obj_name):
        """Save KITTI label file containing 3D bounding box information

        Parameters
        ----------
        start_locs : np.array
            Array of drone starting locations
        id : int
            Drone ID
        camera_info : AirSim CameraInfo
            Camera info struct
        pos2 : AirSim Pose
            Pose of target drone
        label_num : int
            Label number
        obj_name : str
            Object name

        Generates
        ---------
        .txt file
            File containing KITTI-formatted bounding box data of target drone

        """
        ct = get_transform(camera_info.pose, inv=True)
        ct = np.dot(self.ned2cam,ct)

        T2 = get_transform(pos2, inv=False, isDronePose=True)
        t1 = np.append((start_locs[id,:] - start_locs[0,:]),0)
        init_transformmat = np.vstack((np.hstack((np.eye(3),np.reshape(t1,(3,1)))),[0,0,0,1]))

        final_mat = np.dot(ct, np.dot(init_transformmat,T2))

        data = np.zeros((14,))
        data[0] = 0  # truncation goes from 0 to 1
        data[1] = int(0)  # integer (0,1,2,3) indicating occlusion state:
                        # 0 = fully visible, 1 = partly occluded
                        # 2 = largely occluded, 3 = unknown
        data[3:7] = [0.0,0.0,0.0,0.0]    # bbox 2D bounding box of object in the image (0-based index):
                        # contains left, top, right, bottom pixel coordinates
        h = 0.5
        w = 1.5
        l = 1.5
        data[7:10] = [h,w,l]                 # dimensions 3D object dimensions: height, width, length (in meters)
        # location 3D object location x,y,z in camera coordinates (in meters) 
        # and rotation_y Rotation ry around Y-axis in camera coordinates [-pi..pi]
        
        # shift bbox center to be located at ground to align with KITTI format
        data[10:13] = np.dot(final_mat, np.reshape(np.array([0,0,h/2,1]),(4,1)))[0:3,0]

        z_b2 = final_mat[2,3]
        x_b2 = final_mat[0,3]
        data[13] = - np.arctan2(final_mat[2,0], final_mat[0,0])
        heading1 = np.arctan2(z_b2,x_b2)
        alpha = heading1 - data[13] - np.pi/2
        data[2] = alpha    # alpha Observation angle of object, ranging [-pi..pi]
        
        if np.linalg.norm(data[10:13]) <= 25:
            f=open(self.label_folder+("%.6d.txt" % label_num),'a+')
            f.write(obj_name + ' ')
            np.savetxt(f,np.reshape(data,(1,14)),fmt='%0.4f')
            f.close()
        else:
            f=open(self.label_folder+("%.6d.txt" % label_num),'a+')
            f.close()

    
    def save_range(self, start_locs, i, j, pose_i, pose_j, frame):
        """Save range measurement between drone i and drone j

        Parameters
        ----------
        start_locs : np.array
            Array of drone starting locations
        i : int
            First drone ID
        j : int
            Second drone ID
        pose_i : AirSim Pose 
            First drone pose
        pose_j : AirSim Pose 
            Second drone pose
        frame: int
            Frame number

        """
        pos_i_vec = pose_i.kinematics_estimated.position
        pos_j_vec = pose_j.kinematics_estimated.position

        pos_i = np.zeros((3,))
        pos_i[0] = pos_i_vec.x_val + start_locs[i,0]
        pos_i[1] = pos_i_vec.y_val + start_locs[i,1]
        pos_i[2] = pos_i_vec.z_val

        pos_j = np.zeros((3,))
        pos_j[0] = pos_j_vec.x_val + start_locs[j,0]
        pos_j[1] = pos_j_vec.y_val + start_locs[j,1]
        pos_j[2] = pos_j_vec.z_val

        rel_pos = pos_i - pos_j
        inter_range = np.linalg.norm(rel_pos)

        f=open(self.range_folder+("%.6d_%d_%d.txt" % (frame,i,j)),'a+')
        f.write(str(inter_range))
        f.close()


    def read_pickle_file(self, fname):
        """Read Pickle file

        Parameters
        ----------
        fname : str
            File name
    
        Returns
        -------
        content : pickle data 

        """
        file = open(fname, 'rb')
        content = pickle.load(file)
        file.close()
        return content






