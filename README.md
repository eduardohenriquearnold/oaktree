# OakTree
Render large point clouds efficiently using Octrees.

## Dependencies

### Eigen 3.4
This dependency is included in the Docker image, but can be installed with
```bash
git clone --branch 3.4 --depth 1 https://gitlab.com/libeigen/eigen.git
mkdir -p eigen/build_dir
cd eigen/build_dir
cmake ..
sudo make install
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
./demo
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
- [ ] Make all dependencies submodules (Cereal, Eigen)
- [x] Create rectangle example in Python land
- [ ] Test render in large scene (python-land)
- [ ] Fix black line artifacts near node edges
- [ ] Create Pypi package
- [ ] Push publicly to Github

## Sample data
[Sample data](http://kos.informatik.uni-osnabrueck.de/3Dscans/)