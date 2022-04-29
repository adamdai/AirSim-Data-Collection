"""Utilities for converting poses and transforming coordinate frames.

"""

import numpy as np


def write_to_label_file(data,obj_name,file_name):
    f=open(file_name,'a')
    f.write(obj_name + ' ')
    np.savetxt(f,np.reshape(data,(1,14)),fmt='%0.4f')
    f.close()

def get_items_of_a_label_row(my_pose, obj_pose, rel_pose_mat, cam_orientation_mat):
    data = np.zeros((14,))
    data[0] = 0  # truncation goes from 0 to 1
    data[1] = int(0)  # integer (0,1,2,3) indicating occlusion state:
                    # 0 = fully visible, 1 = partly occluded
                    # 2 = largely occluded, 3 = unknown
    data[3:7] = [0.0,0.0,0.0,0.0]    # bbox 2D bounding box of object in the image (0-based index):
                    # contains left, top, right, bottom pixel coordinates
    data[7:10] = [0.6,0.15,0.6]                 # dimensions 3D object dimensions: height, width, length (in meters)
    # location 3D object location x,y,z in camera coordinates (in meters) 
    # and rotation_y Rotation ry around Y-axis in camera coordinates [-pi..pi]
    data[10:14], alpha = get_loc_in_cam_frame(my_pose, obj_pose, rel_pose_mat, cam_orientation_mat)  
    data[2] = alpha    # alpha Observation angle of object, ranging [-pi..pi]
    return data

def get_loc_in_cam_frame(pos1, pos2, init_p1_to_p2, body_to_cam):
    translation1_vec = pos1.kinematics_estimated.position
    translation1 = np.zeros((3,))
    translation1[0] = translation1_vec.x_val
    translation1[1] = translation1_vec.y_val
    translation1[2] = translation1_vec.z_val
    q1 = pos1.kinematics_estimated.orientation
    phi1, theta1, psi1 = quaternion_to_eul(q1)
    rotmat_b_to_n1 = eul_to_rotmat(phi1,theta1,psi1)
    translation2_vec = pos2.kinematics_estimated.position
    translation2 = np.zeros((3,))
    translation2[0] = translation2_vec.x_val
    translation2[1] = translation2_vec.y_val
    translation2[2] = translation2_vec.z_val
    q2 = pos2.kinematics_estimated.orientation
    phi2, theta2, psi2 = quaternion_to_eul(q2)
    rotmat_b_to_n2 = eul_to_rotmat(phi2,theta2,psi2)
    transform_pose2 = make_transform(rotmat_b_to_n2,translation2)
    transform_to_n1 = np.dot(init_p1_to_p2,transform_pose2)
    transform_pose1_inv = (make_transform_inv(rotmat_b_to_n1,translation1))
    transform_to_b1 = np.dot(transform_pose1_inv,transform_to_n1)
    transform_cam1_inv = np.linalg.inv(body_to_cam)
    transform_to_cam1 = np.dot(transform_cam1_inv,transform_to_b1)
    return_array = np.zeros((4,))
    return_array[0:3] = transform_to_cam1[0:3,3]
    #return_array[3] = rotmat_to_euly(transform_to_cam1[0:3,0:3])
    z_b2 = return_array[2]
    x_b2 = return_array[0]
    return_array[3] = - np.arctan2(transform_to_cam1[2,0], transform_to_cam1[0,0])
    heading1 = np.arctan2(z_b2,x_b2)
    alpha = heading1 - return_array[3] - np.pi/2
    return return_array, alpha

def get_transform(pose, inv=False, isDronePose=False):
    if isDronePose:
        translation_vec = pose.kinematics_estimated.position
    else:
        translation_vec = pose.position
    translation = np.zeros((3,))
    translation[0] = translation_vec.x_val
    translation[1] = translation_vec.y_val
    translation[2] = translation_vec.z_val
    if isDronePose:
        q = pose.kinematics_estimated.orientation
    else:
        q = pose.orientation
    phi, theta, psi = quaternion_to_eul(q)
    rotmat_b_to_n = eul_to_rotmat(phi,theta,psi)
    if inv:
        transform_pose = make_transform_inv(rotmat_b_to_n,translation)
    else:
        transform_pose = make_transform(rotmat_b_to_n,translation)
    return transform_pose

# (phi, theta, psi) - (roll, pitch, yaw)
# returns in radians?
def quaternion_to_eul(q):
    q0 = q.w_val
    q1 = q.x_val
    q2 = q.y_val
    q3 = q.z_val
    phi = np.arctan2(2*(q0 * q1 + q2 * q3), 1 - 2*((q1)**2 + (q2)**2))
    theta = np.arcsin(2*(q0 * q2 - q3 * q1))
    psi = np.arctan2(2*(q0 * q3 + q1 * q2), 1 - 2 * ((q2)**2 + (q3)**2))
    return phi, theta, psi

def eul_to_rotmat(phi,theta,psi):
    matx = np.array([[1,0,0],[0,np.cos(phi),-np.sin(phi)],[0,np.sin(phi),np.cos(phi)]])
    maty = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
    matz = np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])
    return np.dot(np.dot(matz,maty),matx)

def rotmat_to_euly(R): ## returns theta between pi and -pi
    if (R[2,0] != 1 and R[2,0] != -1):
        theta1 = -np.arcsin(R[2,0])
        theta2 = np.pi - theta1
        theta = theta1 
    else:
        if R[2,0] == -1:
            theta = np.pi/2
        else:
            theta = -np.pi/2
    return theta

def make_transform(rotmat,t):
    return np.vstack((np.hstack((rotmat,np.reshape(t,(3,1)))),[0,0,0,1]))

def make_transform_inv(rotmat,t):
    return np.vstack((np.hstack((np.transpose(rotmat),-np.dot(np.transpose(rotmat),np.reshape(t,(3,1))))),[0,0,0,1]))

def airsimpos2np(pos):
    return np.array([pos.x_val, pos.y_val, pos.z_val])

# -pi to pi wrapped angle subtraction
def angle_diff(theta1, theta2):
    diff = theta1 - theta2
    diff = (diff + 180) % 360 - 180
    return diff

# #####
# num_files = 1 # number of time steps
# N = 1 # 1 other drone
# start_locs = np.array([[0,0], [5,0]])
# cam_matrix = make_transform(np.array([[0,0,1],[0,-1,0],[1,0,0]]),np.array([0.5,0,0.1]))
# os.chdir('/home/navlab-admin/AirSim-MOT/data')
# datapath = os.getcwd()
# foldername = '/test'
# datafolder = datapath + foldername + '/pose/'
# savefolder = datapath + foldername + '/processed/label/'
# #####

# start0 = start_locs[0,:]
# transform_matrix_list = np.zeros((N,4,4))

# for i in range(N):
#     starti = start_locs[i+1,:]
#     ti = np.append((starti - start0),0)
#     transformi = make_transform(np.eye(3),ti)
#     transform_matrix_list[i,:,:] = transformi

# for i in range(num_files):
#     fname = num_files*[None]
#     fname[0] = 'Drone0' + ("pose%.6d.txt" % i)
#     drone0 = open(datafolder+fname[0], 'rb')
#     pos0 = pickle.load(drone0)
#     drone0.close()

#     for j in range(N):
#         fname[j] = 'Drone' + str(j+1) + ("pose%.6d.txt" % i)

#     for j in range(N):
#         dronej = open(datafolder+fname[j], 'rb')
#         posj = pickle.load(dronej)
#         transform_init = transform_matrix_list[j,:,:]
#         dataj = get_items_of_a_label_row(pos0, posj, transform_init, cam_matrix)
#         write_to_label_file(dataj,'Car',savefolder+("%.6d.txt" % i))
#         dronej.close()