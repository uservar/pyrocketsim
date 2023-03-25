#include <pybind11/pybind11.h>

#include "../RocketSim/src/Sim/BoostPad/BoostPad.h"

namespace py = pybind11;
using namespace pybind11::literals;


void init_boostpad(py::module_ &m) {

    py::class_<BoostPadState>(m, "BoostPadState")
        .def(py::init<>())
        .def_readwrite("is_active", &BoostPadState::isActive)
        .def_readwrite("cooldown", &BoostPadState::cooldown)
        .def("__str__", [](const BoostPadState &padState) {
            return py::str("{{is_active: {},\n cooldown: {}}}").format(
                padState.isActive, padState.cooldown);
        })
        .def("__repr__", [](const BoostPadState &padState) {
            return py::str("<BoostPadState: {}>").format(padState);
        });

    py::class_<BoostPad>(m, "BoostPad")
        .def_readonly("is_big", &BoostPad::isBig)
        .def_readonly("pos", &BoostPad::pos)
        .def("get_state", &BoostPad::GetState)
        .def("set_state", &BoostPad::SetState, "boost_pad_state"_a);

}