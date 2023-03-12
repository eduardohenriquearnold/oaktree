## Things I want to do

### Urgently
- [x] Check depth correctness
- [x] Check render intrinsics with larger resolution
- [x] Parallelise ray tracing with [OMP](https://bisqwit.iki.fi/story/howto/openmp/)
- [x] Resize node dimensions to fit only points within it (upon construction)
- [ ] Parallelise octree creation
- [ ] Ensure all works with non-regular input clouds (non-cubic)

### At some point
- [ ] Serialise Octree with [cereal](https://uscilab.github.io/cereal/)
- [ ] Load point cloud from LAS 
- [ ] Python bindings with [nanobind](https://github.com/wjakob/nanobind)
- [ ] Replace linalg.h with [MathFu](https://github.com/google/mathfu), Google's SIMD compiled vector/matrix library