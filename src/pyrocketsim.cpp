#include <string>
#include <filesystem>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/functional.h>
#include "../RocketSim/src/RocketSim.h"

namespace py = pybind11;
using namespace pybind11::literals;


PYBIND11_MODULE(pyrocketsim, m) {

    m.def("init", &RocketSim::Init);

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

        .def("__getitem__", [](Vec &vec, uint32_t index) {
            if (0 <= index && index < 3)
                return vec[index];
            throw py::index_error("list index out of range");
        })

        .def("__setitem__", [](Vec &vec, uint32_t index, float val) {
            if (0 <= index && index < 3)
                vec[index] = val;
            else throw py::index_error("list index out of range");
        })

        .def(py::self < py::self)
        .def(py::self > py::self)

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

        .def(-py::self)

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
        })

        .def("__format__", [](const Vec& vec, const char* spec) {
            return py::str("[{1:{0}}, {2:{0}}, {3:{0}}]").format(spec,
                vec.x, vec.y, vec.z);
        })

        .def("__str__", [](const Vec &vec) {
            return py::str("[{}, {}, {}]").format(vec.x, vec.y, vec.z);
        })

        .def("__repr__", [](const Vec &vec) {
            return py::str("<Vec: {}>").format(vec);
        });

    py::class_<RotMat>(m, "RotMat")
        .def(py::init<>())
        .def(py::init<Vec, Vec, Vec>(), "forward"_a, "right"_a, "up"_a)
        .def_static("get_identity", &RotMat::GetIdentity)

        .def("__getitem__", [](RotMat &mat, int index) {
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

        .def(py::self * float())
        .def(py::self / float())

        .def(py::self *= float())
        .def(py::self /= float())

        .def("dot", &RotMat::Dot, "vec"_a)
        .def("transpose", &RotMat::Transpose)

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
        })

        .def("__format__", [](const RotMat& mat, const char* spec) {
            return py::str("(FRU) [\n {1:{0}},\n {2:{0}},\n {3:{0}}]").format(spec,
                mat.forward, mat.right, mat.up);
        })

        .def("__str__", [](const RotMat &mat) {
            return py::str("{}").format(mat);
        })

        .def("__repr__", [](const RotMat &mat) {
            return py::str("<RotMat: {}>").format(mat);
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

    py::class_<decltype(CarState::worldContact)>(m, "WorldContact")
        .def_readwrite("has_contact", &decltype(CarState::worldContact)::hasContact)
        .def_readwrite("contact_normal", &decltype(CarState::worldContact)::contactNormal)
        .def("__format__", [](const decltype(CarState::worldContact) &worldContact, const char* spec) {
            return py::str("{{has_contact: {1},\n contact_normal: {2:{0}}}}").format(spec,
                worldContact.hasContact, worldContact.contactNormal);
        })
        .def("__str__", [](const decltype(CarState::worldContact) &worldContact) {
            return py::str("{}").format(worldContact);
        })
        .def("__repr__", [](const decltype(CarState::worldContact) &worldContact) {
            return py::str("<WorldContact: {}>").format(worldContact);
        });

    py::class_<decltype(CarState::carContact)>(m, "CarContact")
        .def_readwrite("other_car_id", &decltype(CarState::carContact)::otherCarID)
        .def_readwrite("cooldown_timer", &decltype(CarState::carContact)::cooldownTimer)
        .def("__format__", [](const decltype(CarState::carContact) &carContact, const char* spec) {
            return py::str("{{other_car_id: {1},\n cooldown_timer: {2:{0}}}}").format(spec,
                carContact.otherCarID, carContact.cooldownTimer);
        })
        .def("__str__", [](const decltype(CarState::carContact) &carContact) {
            return py::str("{}").format(carContact);
        })
        .def("__repr__", [](const decltype(CarState::carContact) &carContact) {
            return py::str("<CarContact: {}>").format(carContact);
        });

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

    py::class_<CarState>(m, "CarState")
        .def(py::init<>())
        .def_readwrite("pos", &CarState::pos)
        .def_readwrite("rot_mat", &CarState::rotMat)
        .def_readwrite("vel", &CarState::vel)
        .def_readwrite("ang_vel", &CarState::angVel)
        
        .def_property_readonly("angles", [](const CarState &car_state) {
            return Angle().FromRotMat(car_state.rotMat);
        })
        
        .def_readwrite("is_on_ground", &CarState::isOnGround)
        .def_readwrite("has_jumped", &CarState::hasJumped)
        .def_readwrite("has_double_jumped", &CarState::hasDoubleJumped)
        .def_readwrite("has_flipped", &CarState::hasFlipped)
        .def_readwrite("last_rel_dodge_torque", &CarState::lastRelDodgeTorque)
        .def_readwrite("jump_time", &CarState::jumpTime)
        .def_readwrite("flip_time", &CarState::flipTime)
        .def_readwrite("is_jumping", &CarState::isJumping)
        .def_readwrite("air_time_since_jump", &CarState::airTimeSinceJump)
        .def_readwrite("boost", &CarState::boost)
        .def_readwrite("time_spent_boosting", &CarState::timeSpentBoosting)
        .def_readwrite("is_supersonic", &CarState::isSupersonic)
        .def_readwrite("supersonic_time", &CarState::supersonicTime)
        .def_readwrite("handbrake_val", &CarState::handbrakeVal)
        .def_readwrite("is_auto_flipping", &CarState::isAutoFlipping)
        .def_readwrite("auto_flip_timer", &CarState::autoFlipTimer)
        .def_readwrite("auto_flip_torque_scale", &CarState::autoFlipTorqueScale)
        
        .def_readwrite("world_contact", &CarState::worldContact)
        .def_readwrite("car_contact", &CarState::carContact)
        
        .def_readwrite("is_demoed", &CarState::isDemoed)
        .def_readwrite("demo_respawn_timer", &CarState::demoRespawnTimer)
        .def_readwrite("last_hit_ball_tick", &CarState::lastHitBallTick)
        
        .def_readwrite("last_controls", &CarState::lastControls);

    py::enum_<Team>(m, "Team")
        .value("BLUE", Team::BLUE)
        .value("ORANGE", Team::ORANGE)
        .export_values();

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

    py::class_<Car>(m, "Car")
        .def("get_config", [](const Car &car) {return car.config;})
        .def_readonly("team", &Car::team)
        .def_readonly("id", &Car::id)
        .def("get_controls", [](const Car &car) {return car.controls;})
        .def("set_controls", [](Car &car, CarControls &controls) {
            car.controls = controls; }, "car_controls"_a)
        .def("get_state", &Car::GetState)
        .def("set_state", &Car::SetState, "car_state"_a)
        .def("demolish", &Car::Demolish)
        .def("respawn", &Car::Respawn, "seed"_a = -1)
        .def_property_readonly("forward_dir", &Car::GetForwardDir)
        .def_property_readonly("right_dir", &Car::GetRightDir)
        .def_property_readonly("up_dir", &Car::GetUpDir);

    py::enum_<GameMode>(m, "GameMode")
        .value("SOCCAR", GameMode::SOCCAR)
        .export_values();

    py::class_<Arena>(m, "Arena")
        .def(py::init(&Arena::Create), "game_mode"_a = GameMode::SOCCAR, "tick_rate"_a = 120)
        .def_readonly("game_mode", &Arena::gameMode)
        .def_readonly("tick_time", &Arena::tickTime)
        .def_property_readonly("tick_rate", &Arena::GetTickRate)
        .def_readonly("tick_count", &Arena::tickCount)
        .def_readonly("ball", &Arena::ball)
        .def("get_boost_pads", &Arena::GetBoostPads, py::return_value_policy::reference)
        .def("get_cars", &Arena::GetCars, py::return_value_policy::reference)
        .def("add_car", &Arena::AddCar, "team"_a, "config"_a = CAR_CONFIG_OCTANE,
            py::return_value_policy::reference)
        .def("remove_car", &Arena::RemoveCar, "car"_a)
        .def("get_car_from_id", &Arena::GetCarFromID, "id"_a)

        .def("set_goal_score_call_back", [](Arena& arena, pybind11::function callback_fn) {
            GoalScoreEventFn callback = [callback_fn](Arena* arena,
                Team scoringTeam, void* userInfo) {
                callback_fn(arena, scoringTeam);
            };
            arena.SetGoalScoreCallback(callback);
        })

        .def("write_to_file", [](Arena &arena, std::string path_str) {
            const std::filesystem::path path = std::filesystem::u8path(path_str);
            arena.WriteToFile(path);
        }, "path_str"_a)

        .def_static("load_from_file", [](std::string path_str) {
            const std::filesystem::path path = std::filesystem::u8path(path_str);
            return Arena::LoadFromFile(path);
        }, "path_str"_a, py::return_value_policy::reference)

        .def("step", &Arena::Step)
        .def("clone", &Arena::Clone, "copy_callbacks"_a)
        .def("reset_to_random_kickoff", &Arena::ResetToRandomKickoff, "seed"_a = -1);
}
