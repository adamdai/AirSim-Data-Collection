"""General utility functions

"""

import numpy as np 
from scipy.spatial.transform import Rotation

from airsim import Vector3r, Quaternionr, Pose


def normalize(v):
    """Normalize numpy vector

    Parameters
    ----------
    v : np.array 
        Vector to normalize

    Returns
    -------
    np.array 
        Normalized vector

    """
    return v / np.linalg.norm(v)


def airsimVec2npArr(vec):
    """Convert an airsim Vector3r object to np array

    Parameters
    ----------
    vec : airsim Vector3r
        Input vector

    Returns
    -------
    np.array
        Converted vector
    
    """
    npArr = np.zeros((3,))
    npArr[0] = vec.x_val
    npArr[1] = vec.y_val
    npArr[2] = vec.z_val
    return npArr


def quat_to_R(quat):
    """Convert quaternion to 3D rotation matrix 

    Parameters
    ----------
    quat : np.array (1 x 4)
        Quaternion in scalar-last (x, y, z, w) format

    Returns
    -------
    np.array (3 x 3)
        Rotation matrix

    """
    q = np.array([quat.x_val, quat.y_val, quat.z_val, quat.w_val])
    r = Rotation.from_quat(q)
    return r.as_matrix()


def R_to_quat(R):
    """Convert 3D rotation matrix to quaternion

    Parameters
    ----------
    R : np.array (3 x 3)
        Rotation matrix

    Returns
    -------
    np.array (1 x 4)
        Quaternion in scalar-last (x, y, z, w) format

    """
    r = Rotation.from_matrix(R)
    q = r.as_quat()
    return Quaternionr(q[0], q[1], q[2], q[3])