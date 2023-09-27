#include "node.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/eigen/dense.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace nanobind
{
    namespace detail
    {
        template <>
        struct type_caster<ImageTensor>
        {
            using NDArray = nb::ndarray<nb::numpy, float, nb::shape<nb::any, nb::any, nb::any>, nb::c_contig, nb::device::cpu>;
            using NDArrayCaster = make_caster<NDArray>;

            static constexpr auto Name = NDArrayCaster::Name;
            template <typename T_> using Cast = ImageTensor;

            static handle from_cpp(const ImageTensor& t, rv_policy policy, cleanup_list* cleanup) noexcept {
                ImageTensor* image = new ImageTensor(t);
                capsule owner(image, [](void* p) noexcept {
                    delete (ImageTensor*)p;
                    });

                size_t shape[3] = { image->height, image->width, image->channels };
                return NDArrayCaster::from_cpp(NDArray(image->pixels.data(), 3, shape, owner), rv_policy::reference, cleanup);
            };

            bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept = delete;

        };
    }
}

NB_MODULE(_oaktree, m) {
    nb::class_<Node>(m, "Node")
        .def(nb::init<unsigned int, const doubleX3&, const floatX3&>(), "max_points_per_node"_a, "points"_a, "points_rgb"_a = Eigen::MatrixX3f(), "Builds Octree from 3D points with optional RGB values")
        .def(nb::init<std::filesystem::path>(), "path"_a, "Loads saved Octree")
        .def("save", &Node::save, "path"_a, "Saves Octree into file")
        .def("test", &Node::test, "Test all points are within node bounds")
        .def("len", &Node::len, "Return tuple with total (#nodes, #levels, #pts)")
        .def("render", &Node::render, "K"_a, "cam2world"_a, "image_hw"_a, "pixel_dilation"_a = 4, "Render point cloud");
}