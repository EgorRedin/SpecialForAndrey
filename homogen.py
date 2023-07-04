
import numpy as np
from math import *
from scipy.spatial.transform import Rotation as R
def matrixFromVectors(tvec, rvec):
    rmat = R.from_rotvec(rvec).as_matrix()
    tvec = np.array([tvec]).T
    out = np.append(rmat, tvec, axis=1)
    out = np.append(out, [[0,0,0,1]], axis=0)
    return out