## Things I want to do

### Urgently
- [x] Check depth correctness
- [x] Check render intrinsics with larger resolution
- [x] Parallelise ray tracing with [OMP](https://bisqwit.iki.fi/story/howto/openmp/)
- [x] Resize node dimensions to fit only points within it (upon construction)
- [x] Parallelise octree creation
- [x] Ensure all works with non-regular input clouds (non-cubic)

### At some point
- [ ] Load point cloud from LAS/xyz formats
- [ ] Serialise Octree with [cereal](https://uscilab.github.io/cereal/)
- [ ] Python bindings with [nanobind](https://github.com/wjakob/nanobind)
- [ ] Replace linalg.h with [MathFu](https://github.com/google/mathfu), Google's SIMD compiled vector/matrix library

### Try some real data
[Sample data](http://kos.informatik.uni-osnabrueck.de/3Dscans/)