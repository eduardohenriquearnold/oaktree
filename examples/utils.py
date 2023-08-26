import numpy as np


def look_at(eye, center, up=None):
    if up is None:
        up = np.array([0.0, 0.0, 1.0])

    z = center - eye
    z /= np.linalg.norm(z)
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)

    mat = np.eye(4)
    mat[:3, 0] = x
    mat[:3, 1] = y
    mat[:3, 2] = z
    mat[:3, 3] = eye
    return mat
