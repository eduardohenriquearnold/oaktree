import sys
sys.path.append("/workspaces/octree/build")

from pathlib import Path
import numpy as np
import pandas as pd
from PIL import Image
from matplotlib import cm

from oaktree import Node

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

SAVE_PATH = Path("octree.bin")

def load_octree_and_render():
    node = Node(path=SAVE_PATH)
    node.stats()
    node.test()

    image_hw = (720, 540)
    K = np.array([[1800, 0, 270], [0, 1800, 360], [0, 0, 1]])
    eye = np.array([-5.5, 10.5, 1.5])
    eye = np.array([-8.5, 7.5, 2.5])
    center = np.zeros(3)
    pose = look_at(eye=eye, center=center, up=np.array([0, 0, 1]))

    rendered = node.render(K=K, cam2world=pose, image_hw=image_hw, pixel_dilation=2)
    print(type(rendered))
    print(rendered.shape)
    print(rendered.dtype)

    rgb = Image.fromarray(np.uint8(rendered[..., :3] * 255))
    depth = rendered[..., 3]
    normalised_depth = (depth - depth.min()) / (depth.max() - depth.min())
    normalised_depth = Image.fromarray(np.uint8(cm.turbo(normalised_depth)*255))

    rgb.save("py_rgb.jpg")
    normalised_depth.save("py_depth.png")

    # import matplotlib.pyplot as plt
    # plt.imshow(depth, cmap='turbo', interpolation='nearest')
    # plt.colorbar()
    # plt.savefig("py_colorbar.png")

def create_octree_box_and_save(color:bool=True):
    NUM_POINTS = 1000000
    LENGTH = (3, 0.5, 1)
    MAX_POINTS_PER_NODE = 100

    # create cuboid by sampling uniform points
    points = np.random.uniform(-0.5, 0.5, size=(NUM_POINTS, 3)).astype(np.float64)
    points *= np.array(LENGTH)[None]

    # create RGB
    if color:
        points_rgb = points.copy()
        points_rgb /= np.linalg.norm(points_rgb, axis=1, keepdims=True)
        points_rgb = (points_rgb + 1) / 2
    else:
        points_rgb = None

    # create octree
    node = Node(max_points_per_node=MAX_POINTS_PER_NODE, points=points, points_rgb=points_rgb)
    node.save(path=SAVE_PATH)

def load_point_cloud(path: Path):
    pcl = pd.read_csv(path)
    pcl = pcl.iloc[:, :3].to_numpy().reshape(-1, 3)
    return pcl

def create_octree_from_data(path: Path):
    points = load_point_cloud(path)
    print(f"Loaded point cloud with shape {points.shape}")

    # create octree
    MAX_POINTS_PER_NODE = 1000
    node = Node(max_points_per_node=MAX_POINTS_PER_NODE, points=points)
    node.save(path=SAVE_PATH)

if __name__ == "__main__":
    # create_octree_box_and_save()
    # create_octree_from_data(Path("../sample/data/lecturehall/lecturehall1.pose1.object1.label.csv"))
    load_octree_and_render()