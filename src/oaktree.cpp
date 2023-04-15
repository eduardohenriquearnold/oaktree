#include "octree.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/pair.h>
#include <nanobind/eigen/dense.h>

namespace nb = nanobind;

NB_MODULE(oaktree, m) {
    nb::class_<OctreeNode>(m, "node")
        .def(nb::init<unsigned int, const doubleX3 &, const doubleX3 &>())
        .def(nb::init<std::filesystem::path>())
        .def("save", &OctreeNode::save)
        .def("test", &OctreeNode::test)
        .def("stats", &OctreeNode::stats)
        .def("render", &OctreeNode::render);
}