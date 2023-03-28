#include <pybind11/pybind11.h>

#include "../RocketSim/src/Sim/CarControls.h"

namespace py = pybind11;
using namespace pybind11::literals;


void init_carcontrols(py::module_ &m) {

    py::class_<CarControls>(m, "CarControls")
        .def(py::init([](float throttle, float steer,
            float pitch, float yaw, float roll,
            bool boost, bool jump, bool handbrake) {
            CarControls controls = CarControls();
            controls.throttle = throttle;
            controls.steer = steer;
            controls.pitch = pitch;
            controls.yaw = yaw;
            controls.roll = roll;
            controls.boost = boost;
            controls.jump = jump;
            controls.handbrake = handbrake;
            return controls;
        }),
        "throttle"_a = 0, "steer"_a = 0, "pitch"_a = 0, "yaw"_a = 0, "roll"_a = 0,
        "boost"_a = false, "jump"_a = false, "handbrake"_a = false)

        .def_readwrite("throttle", &CarControls::throttle)
        .def_readwrite("steer", &CarControls::steer)
        .def_readwrite("pitch", &CarControls::pitch)
        .def_readwrite("yaw", &CarControls::yaw)
        .def_readwrite("roll", &CarControls::roll)
        .def_readwrite("boost", &CarControls::boost)
        .def_readwrite("jump", &CarControls::jump)
        .def_readwrite("handbrake", &CarControls::handbrake)
        .def("clamp_fix", &CarControls::ClampFix)

        .def("__format__", [](const CarControls &carControls, const char* spec) {
            std::string format_str = "{{throttle: {1:{0}},\n steer: {2:{0}},";
            format_str += "\n pitch: {3:{0}},\n yaw: {4:{0}},\n roll: {5:{0}},\n ";
            format_str += "boost: {6},\n jump: {7},\n powerslide: {8}}}";
            return py::str(format_str).format(spec,
                carControls.throttle, carControls.steer,
                carControls.pitch, carControls.yaw, carControls.roll,
                carControls.boost, carControls.jump, carControls.handbrake);
        })

        .def("__str__", [](const CarControls &carControls) {
            return py::str("{}").format(carControls);
        })

        .def("__repr__", [](const CarControls &carControls) {
            return py::str("<CarControls: {}>").format(carControls);
        });

}