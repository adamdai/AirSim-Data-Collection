"""General utility functions

"""

import numpy as np 


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

