#include <pybind11/pybind11.h>

#include "../RocketSim/src/Sim/Car/CarConfig/CarConfig.h"

namespace py = pybind11;


void init_carconfig(py::module_ &m) {

    py::class_<WheelPairConfig>(m, "WheelPairConfig")
        .def(py::init<>())
        .def_readwrite("wheel_radius", &WheelPairConfig::wheelRadius)
        .def_readwrite("suspension_rest_length", &WheelPairConfig::suspensionRestLength)
        .def_readwrite("connection_point_offset", &WheelPairConfig::connectionPointOffset);

    py::class_<CarConfig>(m, "CarConfig")
        .def(py::init<>())
        .def_readwrite("hitbox_size", &CarConfig::hitboxSize)
        .def_readwrite("hitbox_pos_offset", &CarConfig::hitboxPosOffset)
        .def_readwrite("front_wheels", &CarConfig::frontWheels)
        .def_readwrite("back_wheels", &CarConfig::backWheels)
        .def_readwrite("dodge_deadzone", &CarConfig::dodgeDeadzone);

    m.attr("OCTANE") = &CAR_CONFIG_OCTANE;
    m.attr("DOMINUS") = &CAR_CONFIG_DOMINUS;
    m.attr("PLANK") = &CAR_CONFIG_PLANK;
    m.attr("BREAKOUT") = &CAR_CONFIG_BREAKOUT;
    m.attr("HYBRID") = &CAR_CONFIG_HYBRID;
    m.attr("MERC") = &CAR_CONFIG_MERC;

}