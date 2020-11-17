import setup_path 
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2
import time
import math
import pickle

from convert_calib import write_to_label_file
from convert_pose import get_transform, quaternion_to_eul, eul_to_rotmat


class mySensorData:

    def __init__(self, client,  
                 compress_img = False, cam_folder = None, lidar_folder = None, 
                 pose_folder = None, calib_folder = None, label_folder = None,
                 range_folder = None):
        self.client = client
        self.cam_folder = cam_folder # folder to store camera data
        self.lidar_folder = lidar_folder # folder to store lidar data
        self.pose_folder = pose_folder # folder to store the pose data
        self.calib_folder = calib_folder # folder to store the calib data
        self.compress_img = compress_img
        self.label_folder = label_folder
        self.range_folder = range_folder

        # transformation from AirSim NED camera to conventional camera 
        self.ned2cam = np.array([[0,1,0,0],[0,0,1,0],[1,0,0,0],[0,0,0,1]])

    # collect data from drone
    def collectData(self, vehicle_name, 
			get_cam_data = False, get_lidar_data = False, get_calib_data = False,
			 cam_num = 0, lidar_num = 0, pose_num = 0, calib_num = 0):
        lidar_pose = [None]
        camera_info = [None]

        if get_cam_data:
            camera_info = self.genImg(cam_num, vehicle_name)

        if get_lidar_data:
            lidar_pose,  lidar_points = self.genPC(lidar_num, vehicle_name)

        if get_calib_data:
            self.genCalib(calib_num, vehicle_name, lidar_pose, camera_info)

        self.genPose(pose_num, vehicle_name)


    # stores the camera data in folder cam_folder, with name having idx
    def genImg(self, im_num, vehicle_name): 
        responses = self.getImg(vehicle_name)
        # pprint.pprint(responses[0].camera_position)
        # pprint.pprint(responses[0].camera_orientation)
        filename_img = self.cam_folder + ("%.6d.png" % im_num)
        filename_ts = self.pose_folder + ("cam_ts%.6d.txt" % im_num)
        for idx, response in enumerate(responses):
            if self.compress_img: #png format
                airsim.write_file(os.path.normpath(filename_img), response.image_data_uint8)
            else: #uncompressed array
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
    

     # stores the lidar data in folder lidar_folder, with name having idx
    def genPC(self, im_num, vehicle_name):
        
        points, lidar_pose, time_stamp = self.getPC(vehicle_name)
        
        # save timestamp in pose folder
        filename_ts = self.pose_folder + ("velo_ts%.6d.txt" % im_num)
        op = open(os.path.normpath(filename_ts), 'wb')
        pickle.dump(time_stamp, op)
        op.close()

        # save lidar in velodyne folder as bin 
        filename_pc = self.lidar_folder + ("%.6d.bin" % im_num)
        n_points = points.shape[0]
        # append column of 1.0s for intensity
        final_points = np.hstack((points, np.ones((n_points,1), dtype=np.dtype('f4'))))
        final_points.tofile(filename_pc)
        return lidar_pose, final_points
    
    # save pose to file to be processed later
    def genPose(self, im_num, vehicle_name):
        state = self.client.getMultirotorState(vehicle_name = vehicle_name)
        #pprint.pprint(vehicle_name + " position: " + str(state.kinematics_estimated.position))
        filename_pose = self.pose_folder + vehicle_name + ("pose%.6d.txt" % im_num)
        op = open(os.path.normpath(filename_pose), 'wb')
        pickle.dump(state, op)
        op.close()

    # utility for genImg
    def getImg(self, vehicle_name): 
        if self.compress_img:
            responses = self.client.simGetImages([
                            airsim.ImageRequest("0", airsim.ImageType.Scene)], vehicle_name = vehicle_name) 
        else:
            responses = self.client.simGetImages([
                            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name=vehicle_name)
        return responses


    # utility for genPC
    def getPC(self, vehicle_name): 
        k = 1
        points_list = []
        for i in range(k):
            lidar_data = self.client.getLidarData(vehicle_name = vehicle_name)
            time_stamp = lidar_data.time_stamp
            lidar_pose = lidar_data.pose

            if (len(lidar_data.point_cloud) >= 3):
                points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
                points = np.reshape(points, (int(points.shape[0]/3), 3))
                points_list.append(points)
            time.sleep(0.001)
        
        if k == 1:
            points_all = points_list[0]
        else:
            points_all = np.vstack((points_list[0],points_list[1]))
            for i in range(2,k):
                points_all = np.vstack((points_all,points_list[i]))
        
        pprint.pprint('lidar points: ' + str(points_all.shape))

        return points_all, lidar_pose, time_stamp


    # generate KITTI calib calibration file
    def genCalib(self, calib_num, vehicle_name, lidar_pose, camera_info):
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


    # save KITTI label file containing 3D bounding box information
    def saveLabel(self, start_locs, id, camera_info, pos2, label_num, obj_name):
        
        ct = get_transform(camera_info.pose, inv = True)
        ct = np.dot(self.ned2cam,ct)

        T2 = get_transform(pos2, inv = False, isDronePose = True)
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

    
    # save range measurement between drone i and drone j
    def saveRange(self, start_locs, i, j, pose_i, pose_j, frame):

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


    # utility for reading pickle file
    def read_pickle_file(self,fname):
        drone0 = open(fname, 'rb')
        content = pickle.load(drone0)
        drone0.close()
        return content






