"""Utilities for generating calib label files.

"""

import numpy as np


def write_to_label_file(P0, P1, P2, P3, R0, v2c, i2v,file_name):
    """Utility for writing label file

    """
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