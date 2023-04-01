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
        .def_readwrite("tick_count_when_hit", &BallHitInfo::tickCountWhenHit)

        .def("__format__", [](const BallHitInfo &ballHitInfo, const char* spec) {
            std::string format_str = "{{car_id: {1},\n relative_pos_on_ball: {2:{0}},";
            format_str += "\n ball_pos: {3:{0}},\n extra_hit_vel: {4:{0}},\n";
            format_str += "tick_count_when_hit: {5}}}";
            return py::str(format_str).format(spec,
                ballHitInfo.carID, ballHitInfo.relativePosOnBall, ballHitInfo.ballPos,
                ballHitInfo.extraHitVel, ballHitInfo.tickCountWhenHit);
        })

        .def("__str__", [](const BallHitInfo &ballHitInfo) {
            return py::str("{}").format(ballHitInfo);
        })

        .def("__repr__", [](const BallHitInfo &ballHitInfo) {
            return py::str("<BallHitInfo: {}>").format(ballHitInfo);
        });

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