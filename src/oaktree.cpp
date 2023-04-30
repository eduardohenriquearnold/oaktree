#include "octree.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/pair.h>
#include <nanobind/eigen/dense.h>

namespace nb = nanobind;

namespace nanobind
{
namespace detail
{
    template <>
    struct type_caster<ImageTensor> 
    {
        using NDArray = nb::ndarray<nb::numpy, float, nb::shape<nb::any, nb::any, nb::any>, nb::c_contig, nb::device::cpu>;
        using NDArrayCaster = make_caster<NDArray>;

        // NB_TYPE_CASTER(ImageTensor, NDArrayCaster::Name);
        static constexpr auto Name = NDArrayCaster::Name;
        template <typename T_> using Cast = ImageTensor;

        static handle from_cpp(const ImageTensor &t, rv_policy policy, cleanup_list *cleanup) noexcept {
            ImageTensor *image = new ImageTensor(t);
            capsule owner(image, [](void *p) noexcept {
                delete (ImageTensor *) p;
            });

            size_t shape[3] = {image->height, image->width, image->channels};
            return NDArrayCaster::from_cpp(NDArray(image->pixels.data(), 3, shape, owner), rv_policy::reference, cleanup);
        };

        /// Generating an expression template from a Python object is, of course, not possible
        bool from_python(handle src, uint8_t flags, cleanup_list *cleanup) noexcept = delete;

    };
}
}

NB_MODULE(oaktree, m) {
    nb::class_<OctreeNode>(m, "node")
        .def(nb::init<unsigned int, const doubleX3 &, const doubleX3 &>())
        .def(nb::init<std::filesystem::path>())
        .def("save", &OctreeNode::save)
        .def("test", &OctreeNode::test)
        .def("stats", &OctreeNode::stats)
        .def("render", &OctreeNode::render);
}