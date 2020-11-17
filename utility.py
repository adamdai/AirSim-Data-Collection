# Utility functions:
#   
#   airSimVec2npArr
#
#

import numpy as np 
import os

# Convert an airsim Vector3r object to a 3D numpy array
def airsimVec2npArr(vec):
    npArr = np.zeros((3,))
    npArr[0] = vec.x_val
    npArr[1] = vec.y_val
    npArr[2] = vec.z_val
    return npArr

