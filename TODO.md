## Things I want to do

- [x] Check depth correctness
- [x] Check render intrinsics with larger resolution
- [x] Parallelise ray tracing with [OMP](https://bisqwit.iki.fi/story/howto/openmp/)
- [x] Resize node dimensions to fit only points within it (upon construction)
- [x] Parallelise octree creation
- [x] Ensure all works with non-regular input clouds (non-cubic)
- [x] Ensure correctness for camera within octree

- [x] Split files and use CMake for build
- [ ] Load point cloud from LAS/xyz formats
- [ ] Test render in large scene
- [ ] Extend render function to RGB
- [ ] Serialise Octree with [cereal](https://uscilab.github.io/cereal/)
- [ ] Replace linalg.h with Eigen/something that easily integrates with NumPy
- [ ] Python bindings with [nanobind](https://github.com/wjakob/nanobind)

### Try some real data
[Sample data](http://kos.informatik.uni-osnabrueck.de/3Dscans/)