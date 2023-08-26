from pathlib import Path
import urllib.request
import tarfile

import numpy as np
import pandas as pd
from PIL import Image
from matplotlib import cm
from oaktree import Node

from utils import look_at

def load_wurzburg_lecture_hall():
    path = Path("data/lecturehall/lecturehall1.pose1.object1.label.csv")
    URL = "http://kos.informatik.uni-osnabrueck.de/3Dscans/lecturehall.tar.xz"

    if not path.exists():
        Path("data").mkdir(exist_ok=True)

        # download dataset
        print("Downloading dataset")
        urllib.request.urlretrieve(URL, "data/lecturehall.tar.xz")
        print("Download completed")

        # extract file
        print("Extracting files")
        with tarfile.open("data/lecturehall.tar.xz", 'r:xz') as tar:
            tar.extractall("data")
        print("Extraction completed")

    pcl = pd.read_csv(path)
    pcl = pcl.iloc[:, :3].to_numpy().reshape(-1, 3)
    # original data in cm, we use meters for convenience
    pcl /= 100
    return pcl

def get_camera_poses_circle(center: tuple[float, float, float], radius: float, n: int):
    """Yields n camera poses around a circle with specified center and radius. 
    Cameras focus on central point.
    """
    center = np.array(center).reshape(-1)

    for i in range(n):
        angle = 2 * np.pi * i / n
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        eye = np.array([x, y, center[2]])
        pose = look_at(eye, center)
        yield pose

def main():
    octree_path = Path("data/lecturehall.bin")

    if octree_path.exists():
        # check if Octree binary file already exists and use it
        node = Node(path=octree_path)
        print(f"Loaded Octree from {octree_path}")
    else:
        # otherwise creates the octree and saves it to avoid creating it every time the script is executed
        points_xyz = load_wurzburg_lecture_hall()
        print(f"Loaded point cloud with {points_xyz.shape[0]} points!")

        MAX_POINTS_PER_NODE = 5000
        node = Node(max_points_per_node=MAX_POINTS_PER_NODE, points=points_xyz)
        node.save(path=octree_path)
        print(f"Created Octree from scratch, saved to {octree_path}")

    # test node is created/loaded successfully
    # exception is raised if not
    node.test()

    # print the octree stats
    num_nodes, num_levels, num_pts = node.len()
    print(f"{num_nodes=} {num_levels=} {num_pts=}")

    # define camera parameters
    image_hw = (720, 540)
    K = np.array([[500, 0, 270], [0, 500, 360], [0, 0, 1]])

    # render depth maps for camera in a circle
    for i, pose in enumerate(get_camera_poses_circle(center=np.zeros(3), radius=0.2, n=10)):
        rendered = node.render(K=K, cam2world=pose, image_hw=image_hw, pixel_dilation=3)
        depth = rendered[..., 3]
        normalised_depth = (depth - depth.min()) / (depth.max() - depth.min())
        normalised_depth = Image.fromarray(np.uint8(cm.turbo(normalised_depth)*255))
        filename = f"wurzburg{i:03}.png"
        normalised_depth.save(filename)
        print(f"Rendered {filename}")


if __name__ == "__main__":
    main()