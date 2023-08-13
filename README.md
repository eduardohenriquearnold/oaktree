# OakTree
Render large point clouds efficiently using Octrees. The library is a C++ module with Python bindings for ease of use.

## Quick Demo
Create an Octree from a Numpy array containing points xyz positions and RGB intensities.
```python
import numpy as np
from oaktree import Node

# use random points, but could be any Numpy array with shape [N, 3]
points = np.random.uniform(size=(100000, 3))
points_rgb = np.random.uniform(size=(100000, 3))

# create Node that allows efficient querying
node = Node(max_points_per_node=1000, points=points, points_rgb=points_rgb)

# camera parameters
image_hw = (640, 480)
K = np.array([[300, 0, 320], [0, 300, 240], [0, 0, 1]], dtype=np.float64)
cam2world = np.eye(4)

rgbd = node.render(K=K, cam2world=cam2world, image_hw=image_hw)
rgb = rgbd[:, :, :3]
depth = rgbd[:, :, 3]
```

For more comprehensive examples, including real datasets, check the `examples` folder.
Note: examples require extra dependencies including `matplotlib, PIL`.
```bash
cd examples
python box.py
python wurzburg.py
```

## Installation

### Pypi
```bash
pip install oaktree
```

### Build from source
```bash
git clone --recursive https://github.com/eduardohenriquearnold/oaktree.git
cd oaktree
pip install .
```

Unit tests can be executed with
```bash
mkdir build
cd build
cmake ..
cmake --build .
ctest
# or, to see results of individual tests
./cpp_test
```


## TODO
- [ ] Make nice README.md with examples
- [ ] Create Pypi package
- [ ] Push publicly to Github

## Sample data
[Sample data](http://kos.informatik.uni-osnabrueck.de/3Dscans/)