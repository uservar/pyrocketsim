#include <pybind11/pybind11.h>

#include "../RocketSim/src/Sim/Car/CarConfig/CarConfig.h"


enum class CarPreset {
    OCTANE,
    DOMINUS,
    PLANK,
    BREAKOUT,
    HYBRID,
    MERC,
};

CarConfig const *const carConfigPresets[] = {
    &CAR_CONFIG_OCTANE,
    &CAR_CONFIG_DOMINUS,
    &CAR_CONFIG_PLANK,
    &CAR_CONFIG_BREAKOUT,
    &CAR_CONFIG_HYBRID,
    &CAR_CONFIG_MERC,
};


namespace py = pybind11;

CarConfig GetCarConfigFromPreset(uint8_t preset) {
    if (preset < sizeof(carConfigPresets)) {
        return CarConfig(*carConfigPresets[preset]);
    }
    else throw py::index_error("Unknown car preset");
}

void init_carconfig(py::module_ &m) {

    py::class_<WheelPairConfig>(m, "WheelPairConfig")
        .def(py::init<>())
        .def_readwrite("wheel_radius", &WheelPairConfig::wheelRadius)
        .def_readwrite("suspension_rest_length", &WheelPairConfig::suspensionRestLength)
        .def_readwrite("connection_point_offset", &WheelPairConfig::connectionPointOffset);

    py::class_<CarConfig>(m, "CarConfig")
        .def(py::init<>())
        .def(py::init(&GetCarConfigFromPreset))
        .def_readwrite("hitbox_size", &CarConfig::hitboxSize)
        .def_readwrite("hitbox_pos_offset", &CarConfig::hitboxPosOffset)
        .def_readwrite("front_wheels", &CarConfig::frontWheels)
        .def_readwrite("back_wheels", &CarConfig::backWheels)
        .def_readwrite("dodge_deadzone", &CarConfig::dodgeDeadzone)

        .def("__copy__",  [](const CarConfig &self) {
            return CarConfig(self);
        })
        .def("__deepcopy__", [](const CarConfig &self, py::dict) {
            return CarConfig(self);
        }, "memo"_a);

    py::enum_<CarPreset>(m, "CarPreset")
        .value("OCTANE", CarPreset::OCTANE)
        .value("DOMINUS", CarPreset::DOMINUS)
        .value("PLANK", CarPreset::PLANK)
        .value("BREAKOUT", CarPreset::BREAKOUT)
        .value("HYBRID", CarPreset::HYBRID)
        .value("MERC", CarPreset::MERC)
        .export_values();
}