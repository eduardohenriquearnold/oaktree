import sys
sys.path.append("/workspaces/octree/build")

from pathlib import Path
import numpy as np
from PIL import Image
from matplotlib import cm

import oaktree

def look_at(eye, center, up):
    z = (center - eye)
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


path = Path("octree.bin")
node = oaktree.node(path)
node.stats()
node.test()

image_hw = (720, 540)
K = np.array([[500, 0, 270], [0, 500, 360], [0, 0, 1]])
eye = np.array([-1.5, 1.5, -1.5])
pose = look_at(eye, np.zeros(3), np.array([0, -1, 0]))

rendered = node.render(K, pose, image_hw)
print(type(rendered))
print(rendered.shape)
print(rendered.dtype)

rgb = Image.fromarray(np.uint8(rendered[..., :3] * 255))
depth = rendered[..., 3]
normalised_depth = (depth - depth.min()) / (depth.max() - depth.min())
normalised_depth = Image.fromarray(np.uint8(cm.turbo(normalised_depth)*255))

rgb.save("py_rgb.jpg")
normalised_depth.save("py_depth.png")