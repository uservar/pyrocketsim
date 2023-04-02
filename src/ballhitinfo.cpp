#include <pybind11/pybind11.h>

#include "../RocketSim/src/Sim/BallHitInfo/BallHitInfo.h"

namespace py = pybind11;
using namespace pybind11::literals;


void init_ballhitinfo(py::module_ &m) {

    py::class_<BallHitInfo>(m, "BallHitInfo")
        .def(py::init<>())
        .def_readwrite("is_valid", &BallHitInfo::isValid)
        .def_readwrite("relative_pos_on_ball", &BallHitInfo::relativePosOnBall)
        .def_readwrite("ball_pos", &BallHitInfo::ballPos)
        .def_readwrite("extra_hit_vel", &BallHitInfo::extraHitVel)
        .def_readwrite("tick_count_when_hit", &BallHitInfo::tickCountWhenHit)
        .def_readwrite("tick_count_when_extra_impulse_applied",
        	&BallHitInfo::tickCountWhenExtraImpulseApplied)

        .def("__format__", [](const BallHitInfo &ballHitInfo, const char* spec) {
            std::string format_str = "{{is_valid: {1},\n relative_pos_on_ball: {2:{0}},";
            format_str += "\n ball_pos: {3:{0}},\n extra_hit_vel: {4:{0}},";
            format_str += "\n tick_count_when_hit: {5},";
            format_str += "\n tick_count_when_extra_impulse_applied: {6}}}";
            return py::str(format_str).format(spec,
                ballHitInfo.isValid, ballHitInfo.relativePosOnBall, ballHitInfo.ballPos,
                ballHitInfo.extraHitVel, ballHitInfo.tickCountWhenHit,
                ballHitInfo.tickCountWhenExtraImpulseApplied);
        })

        .def("__str__", [](const BallHitInfo &ballHitInfo) {
            return py::str("{}").format(ballHitInfo);
        })

        .def("__repr__", [](const BallHitInfo &ballHitInfo) {
            return py::str("<BallHitInfo: {}>").format(ballHitInfo);
        });

}