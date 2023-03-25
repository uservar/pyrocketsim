#include <pybind11/pybind11.h>

#include "../RocketSim/src/Sim/Ball/Ball.h"

namespace py = pybind11;
using namespace pybind11::literals;


void init_ball(py::module_ &m) {

    py::class_<BallHitInfo>(m, "BallHitInfo")
        .def(py::init<>())
        .def_readwrite("car_id", &BallHitInfo::carID)
        .def_readwrite("relative_pos_on_ball", &BallHitInfo::relativePosOnBall)
        .def_readwrite("ball_pos", &BallHitInfo::ballPos)
        .def_readwrite("extra_hit_vel", &BallHitInfo::extraHitVel)
        .def_readwrite("tick_count_when_hit", &BallHitInfo::tickCountWhenHit);

    py::class_<BallState>(m, "BallState")
        .def(py::init<>())
        .def_readwrite("pos", &BallState::pos)
        .def_readwrite("vel", &BallState::vel)
        .def_readwrite("ang_vel", &BallState::angVel)
        .def_readwrite("ball_hit_info", &BallState::ballHitInfo);

    py::class_<Ball>(m, "Ball")
        .def("get_state", &Ball::GetState)
        .def("set_state", &Ball::SetState, "ball_state"_a)
        .def("get_radius", &Ball::GetRadius);

}