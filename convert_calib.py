import numpy as np
import pickle
import pprint
import sys
import os

def write_to_label_file(P0, P1, P2, P3, R0, v2c, i2v,file_name):
    f=open(file_name,'a')
    f.write('P0: ')
    np.savetxt(f,np.reshape(P0,(1,12)))
    f.write('P1: ')
    np.savetxt(f,np.reshape(P1,(1,12)))
    f.write('P2: ')
    np.savetxt(f,np.reshape(P2,(1,12)))
    f.write('P3: ')
    np.savetxt(f,np.reshape(P3,(1,12)))
    f.write('R0_rect: ')
    np.savetxt(f,np.reshape(R0,(1,9)))
    f.write('Tr_velo_to_cam: ')
    np.savetxt(f,np.reshape(v2c,(1,12)))
    f.write('Tr_imu_to_velo: ')
    np.savetxt(f,np.reshape(i2v,(1,12)))
    f.close()

# def make_transform(rotmat,t):
#     return np.vstack((np.hstack((rotmat,np.reshape(t,(3,1)))),[0,0,0,1]))

# #####
# num_files = 1 # number of time steps
# #cam_matrix = make_transform(np.array([[0,0,1],[0,-1,0],[1,0,0]]),np.array([0.5,0.0,0.1]))
# cam_matrix = make_transform(np.array([[1,0,0],[0,1,0],[0,0,1]]),np.array([0.5,0.0,0.1]))
# velo_matrix = make_transform(np.array([[1,0,0],[0,1,0],[0,0,1]]),np.array([0.0,0.0,0.0]))
# os.chdir('/home/navlab-admin/AirSim-MOT/data')
# datapath = os.getcwd()
# foldername = '/test'
# savefolder = datapath + foldername + '/processed/calib/'
# #####

# velo_to_cam_mat = np.dot(np.linalg.inv(cam_matrix),velo_matrix)
# imu_to_velo_mat = np.linalg.inv(velo_matrix)
# v2c = velo_to_cam_mat[0:3,:]
# i2v = imu_to_velo_mat[0:3,:]
# R0 = np.eye(3)
# P0 = np.hstack((np.eye(3),np.zeros((3,1))))
# P1 = P0
# P2 = P0
# P3 = P0
# for i in range(num_files):
#     write_to_label_file(P0, P1, P2, P3, R0, v2c, i2v, savefolder+("%.6d.txt" % i))