#include <pybind11/pybind11.h>
#include <pybind11/stl/filesystem.h>

#include "../src/mathtypes.cpp"
#include "../src/boostpad.cpp"
#include "../src/carcontrols.cpp"
#include "../src/carconfig.cpp"
#include "../src/ball.cpp"
#include "../src/car.cpp"
#include "../src/mutatorconfig.cpp"
#include "../src/arena.cpp"

#include "../RocketSim/src/RocketSim.h"

namespace py = pybind11;
using namespace pybind11::literals;


PYBIND11_MODULE(pyrocketsim, m) {

    m.def("init", &RocketSim::Init, "path"_a = "collision_meshes");

    init_mathtypes(m);
    init_boostpad(m);
    init_carcontrols(m);
    init_carconfig(m);
    init_ball(m);
    init_car(m);
    init_mutatorconfig(m);
    init_arena(m);
}
