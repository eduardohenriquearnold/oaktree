# OakTree
Render large point clouds efficiently using Octrees.

## Dependencies

All dependencies are included in submodules inside `external`.

Make sure to download them by using
```shell
git clone https://github.com/eduardohenriquearnold/octree.git
git submodule update --init --recursive
```

## Build
```bash
mkdir build
cd build
cmake ..
cmake --build .
```

## Test demo
```bash
cd build
# cpp
./demo
# python
python3 ../src/demo.py
```

## TODO

- [x] Check depth correctness
- [x] Check render intrinsics with larger resolution
- [x] Parallelise ray tracing with [OMP](https://bisqwit.iki.fi/story/howto/openmp/)
- [x] Resize node dimensions to fit only points within it (upon construction)
- [x] Parallelise octree creation
- [x] Ensure all works with non-regular input clouds (non-cubic)
- [x] Ensure correctness for camera within octree
- [x] Split files and use CMake for build
- [x] Store points/color within OctreeNode, not indices
- [x] Extend render function to RGB

- [x] Replace linalg.h with Eigen
- [x] Serialise Octree with [cereal](https://uscilab.github.io/cereal/)
- [x] Python bindings with [nanobind](https://github.com/wjakob/nanobind)
- [x] Make all dependencies submodules (Cereal, Eigen)
- [x] Create rectangle example in Python land
- [ ] Fix black line artifacts near node edges
    > This happens as node size gets quite small and intersection algorithm fails. 
    > Can be solved by increasing the size of the box, but the epsilon is arbitrary and may be dependent on distance to camera
    > Ideally we want the intersection algorithm to handle this, i.e. it should consider nodes for which the CONE intersects, not only the ray
    > Check cone-box intersection code below:
    > https://stackoverflow.com/questions/22023977/detect-if-a-cube-and-a-cone-intersect-each-other
    >  https://blog.squareys.de/aabb-cone-intersection/
- [ ] Test render in large scene (python-land)
- [ ] C++ formatter
- [ ] Create Pypi package
- [ ] Push publicly to Github

## Sample data
[Sample data](http://kos.informatik.uni-osnabrueck.de/3Dscans/)