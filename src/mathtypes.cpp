#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include "../RocketSim/src/Math/MathTypes/MathTypes.h"

namespace py = pybind11;
using namespace pybind11::literals;


void init_mathtypes(py::module_ &m) {

    py::class_<Vec>(m, "Vec")

        .def(py::init<float, float, float>(), "x"_a = 0, "y"_a = 0, "z"_a = 0)
        //The _a suffix forms a C++11 literal which is equivalent to py::arg()

        .def_readwrite("x", &Vec::x)
        .def_readwrite("y", &Vec::y)
        .def_readwrite("z", &Vec::z)

        .def("is_zero", &Vec::IsZero)
        .def("length_sq", &Vec::LengthSq)
        .def("length", &Vec::Length)
        .def("length", &Vec::Length)
        .def("dot", &Vec::Dot, "other"_a)
        .def("cross", &Vec::Cross, "other"_a)
        .def("dist_sq", &Vec::DistSq, "other"_a)
        .def("dist", &Vec::Dist, "other"_a)
        .def("dist_sq_2d", &Vec::DistSq2D, "other"_a)
        .def("dist_2d", &Vec::Dist2D, "other"_a)
        .def("normalized", &Vec::Normalized)

        .def("__getitem__", [](const Vec &vec, uint32_t index) {
            if (0 <= index && index < 3)
                return vec[index];
            throw py::index_error("list index out of range");
        })

        .def("__setitem__", [](Vec &vec, uint32_t index, float val) {
            if (0 <= index && index < 3)
                vec[index] = val;
            else throw py::index_error("list index out of range");
        })

        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)

        .def(py::self += py::self)
        .def(py::self -= py::self)
        .def(py::self *= py::self)
        .def(py::self /= py::self)

        // some operators were not defined in RocketSim at the time of writing this
        .def(py::self * float())
        .def(py::self / float())

        .def(py::self *= float())
        .def(py::self /= float())

        .def(py::self < py::self)
        .def(py::self > py::self)

        .def(-py::self)

        .def(py::self == py::self)
        .def(py::self != py::self)

        .def("__format__", [](const Vec& vec, const char* spec) {
            return py::str("[{1:{0}}, {2:{0}}, {3:{0}}]").format(spec,
                vec.x, vec.y, vec.z);
        })

        .def("__str__", [](const Vec &vec) {
            return py::str("[{}, {}, {}]").format(vec.x, vec.y, vec.z);
        })

        .def("__repr__", [](const Vec &vec) {
            return py::str("<Vec: {}>").format(vec);
        })

        .def("as_tuple", [](const Vec &vec) {
            return py::make_tuple(vec.x, vec.y, vec.z);
        })

        .def("as_numpy", [](const Vec &vec) {
            py::array_t<float> arr({3});
            auto buf = arr.mutable_data();
            buf[0] = vec.x;
            buf[1] = vec.y;
            buf[2] = vec.z;
            return arr;
        });

    py::class_<RotMat>(m, "RotMat")
        .def(py::init<>())
        .def(py::init<Vec, Vec, Vec>(), "forward"_a, "right"_a, "up"_a)
        .def_static("get_identity", &RotMat::GetIdentity)

        .def("__getitem__", [](const RotMat &mat, int index) {
            if (0 <= index && index < 3)
                return mat[index];
            throw py::index_error("list index out of range");
        })

        .def("__setitem__", [](RotMat &mat, uint32_t index, Vec &vec) {
            if (0 <= index && index < 3)
                mat[index] = vec;
            else throw py::index_error("list index out of range");
        })

        .def(py::self + py::self)
        .def(py::self - py::self)

        .def(py::self += py::self)
        .def(py::self -= py::self)

        .def(py::self * float())
        .def(py::self / float())

        .def(py::self *= float())
        .def(py::self /= float())

        .def(py::self == py::self)
        .def(py::self != py::self)

        .def("dot", &RotMat::Dot, "vec"_a)
        .def("transpose", &RotMat::Transpose)

        .def("__format__", [](const RotMat& mat, const char* spec) {
            return py::str("(FRU) [\n {1:{0}},\n {2:{0}},\n {3:{0}}]").format(spec,
                mat.forward, mat.right, mat.up);
        })

        .def("__str__", [](const RotMat &mat) {
            return py::str("{}").format(mat);
        })

        .def("__repr__", [](const RotMat &mat) {
            return py::str("<RotMat: {}>").format(mat);
        })

        .def("as_numpy", [](const RotMat &mat) {
            py::array_t<float> arr({3, 3});
            auto buf = arr.mutable_data();
            buf[0] = mat.forward.x;
            buf[1] = mat.forward.y;
            buf[2] = mat.forward.z;
            buf[3] = mat.right.x;
            buf[4] = mat.right.y;
            buf[5] = mat.right.z;
            buf[6] = mat.up.x;
            buf[7] = mat.up.y;
            buf[8] = mat.up.z;
            return arr;
        });

    py::class_<Angle>(m, "Angle")
        .def(py::init<float, float, float>(), "yaw"_a = 0, "pitch"_a = 0, "roll"_a = 0)

        .def_readwrite("yaw", &Angle::yaw)
        .def_readwrite("pitch", &Angle::pitch)
        .def_readwrite("roll", &Angle::roll)

        .def_static("from_rot_mat", &Angle::FromRotMat, "rot_mat"_a)
        .def("to_rot_mat", &Angle::ToRotMat)

        .def("get_forward_vector", &Angle::GetForwardVector)
        .def("normalize_fix", &Angle::NormalizeFix)

        .def("as_tuple", [](const Angle &ang) {
            return py::make_tuple(ang.yaw, ang.pitch, ang.roll);
        })

        .def("as_numpy", [](const Angle &ang) {
            py::array_t<float> arr({3});
            auto buf = arr.mutable_data();
            buf[0] = ang.yaw;
            buf[1] = ang.pitch;
            buf[2] = ang.roll;
            return arr;
        })

        .def("__format__", [](const Angle &ang, const char* spec) {
            return py::str("(YPR) [{1:{0}}, {2:{0}}, {3:{0}}]").format(spec,
                ang.yaw, ang.pitch, ang.roll);
        })

        .def("__str__", [](const Angle &ang) {
            return py::str("{}").format(ang);
        })

        .def("__repr__", [](const Angle &ang) {
            return py::str("<Angle: {}>").format(ang);
        });
}