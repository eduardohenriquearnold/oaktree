import numpy as np
from PIL import Image
from matplotlib import cm
from oaktree import Node

from utils import look_at

# use random points, but could be any Numpy array with shape [N, 3]
points = np.random.uniform(size=(100000, 3))

# color based on point position
points_rgb = points.copy()
points_rgb /= np.linalg.norm(points_rgb, axis=1, keepdims=True)
points_rgb = (points_rgb + 1) / 2

# create Node that allows efficient querying
node = Node(max_points_per_node=1000, points=points, points_rgb=points_rgb)

# camera parameters: resolution, intrinsics and pose (homogeneous, transforms points from camera-to-world)
image_hw = (480, 640)
K = np.array([[300, 0, 320], [0, 300, 240], [0, 0, 1]], dtype=np.float64)
cam2world = look_at(eye=np.array([2, 2, 2]), center=np.zeros(3))
print([list(i) for i in cam2world.round(3)])

# render image from specified pose
rgbd = node.render(K=K, cam2world=cam2world, image_hw=image_hw, pixel_dilation=10)
rgb = rgbd[..., :3]
depth = rgbd[..., 3]

# transform RGB (currently in float [0, 1]) to UInt8 and saves
rgb = Image.fromarray(np.uint8(rgb * 255))
rgb.save("box_rgb.jpg")

normalised_depth = (depth - depth.min()) / (depth.max() - depth.min())
normalised_depth = Image.fromarray(np.uint8(cm.turbo(normalised_depth)*255))
normalised_depth.save("box_depth.png")
