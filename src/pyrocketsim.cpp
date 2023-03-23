#include <vector>
#include <string.h>
#include <boost/python.hpp>
#include <boost/format.hpp>
#include <boost/python/numpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "../RocketSim/src/RocketSim.h"

using namespace boost::python;
namespace np = numpy;


// Vec
float VecGetItem(const Vec& vec, uint32_t index) {
    if (index < 0 || index > 2) {
        PyErr_SetString(PyExc_IndexError, "index out of range");
        throw_error_already_set();
    }
    return vec[index];
}

float VecSetItem(Vec& vec, uint32_t index, float value) {
    if (index < 0 || index > 2) {
        PyErr_SetString(PyExc_IndexError, "index out of range");
        throw_error_already_set();
    }
    return vec[index] = value;
}

tuple VecAsTuple(const Vec& vec) {
    return make_tuple(vec.x, vec.y, vec.z);
}

np::ndarray VecAsNumpy(const Vec& vec) {
    np::ndarray arr = np::zeros(make_tuple(3), np::dtype::get_builtin<float>());
    float* data = reinterpret_cast<float*>(arr.get_data());
    data[0] = vec.x;
    data[1] = vec.y;
    data[2] = vec.z;
    return arr;
}

object VecFormat(const Vec& vec, const char* spec = "") {
    return str("[{1:{0}}, {2:{0}}, {3:{0}}]").attr("format")(
        spec, vec.x, vec.y, vec.z);
}


// RotMat
Vec RotMatGetItem(const RotMat& mat, uint32_t index) {
    if (index < 0 || index > 2) {
        PyErr_SetString(PyExc_IndexError, "index out of range");
        throw_error_already_set();
    }
    return mat[index];
}

Vec RotMatSetItem(RotMat& mat, uint32_t index, Vec vec) {
    if (index < 0 || index > 2) {
        PyErr_SetString(PyExc_IndexError, "index out of range");
        throw_error_already_set();
    }
    return mat[index] = vec;
}

object RotMatFormat(const RotMat& mat, const char* spec = "") {
    return str("(FRU) [\n {1:{0}},\n {2:{0}},\n {3:{0}}]").attr("format")(
        spec, mat.forward, mat.right, mat.up);
}

np::ndarray RotMatAsNumpy(const RotMat& mat) {
    np::ndarray arr = np::zeros(make_tuple(3, 3), np::dtype::get_builtin<float>());
    float* data = reinterpret_cast<float*>(arr.get_data());
    data[0] = mat.forward.x;
    data[1] = mat.forward.y;
    data[2] = mat.forward.z;
    data[3] = mat.right.x;
    data[4] = mat.right.y;
    data[5] = mat.right.z;
    data[6] = mat.up.x;
    data[7] = mat.up.y;
    data[8] = mat.up.z;
    return arr;
}

// Angle
tuple AngleAsTuple(const Angle& ang) {
    return make_tuple(ang.yaw, ang.pitch, ang.roll);
}

np::ndarray AngleAsNumpy(const Angle& ang) {
    np::ndarray arr = np::zeros(make_tuple(3), np::dtype::get_builtin<float>());
    float* data = reinterpret_cast<float*>(arr.get_data());
    data[0] = ang.yaw;
    data[1] = ang.pitch;
    data[2] = ang.roll;
    return arr;
}

object AngleFormat(const Angle& ang, const char* spec = "") {
    return str("[{1:{0}}, {2:{0}}, {3:{0}}]").attr("format")(
        spec, ang.yaw, ang.pitch, ang.roll);
}

// BoostPadState
object BoostPadStateFormat(const BoostPadState& bps, const char* spec = "") {
    return str("{{is_active: {},\n cooldown: {}}}").attr("format")(
        spec, bps.isActive, bps.cooldown);
}

// Ball
float BallGetRadius(Ball& ball) {
    return ball.GetRadius() * 50;
}


// WorldContact
object WorldContactFormat(const decltype(CarState::worldContact)& wc, const char* spec = "") {
    return str("{{has_contact: {1},\n contact_normal: {2:{0}}}}").attr("format")(
        spec, wc.hasContact, wc.contactNormal);
}

// CarContact
object CarContactFormat(const decltype(CarState::carContact)& cc, const char* spec = "") {
    return str("{{other_car_id: {1},\n cooldown_timer: {2:{0}}}}").attr("format")(
        spec, cc.otherCarID, cc.cooldownTimer);
}

// CarControls
object CarControlsFormat(const CarControls& carControls, const char* spec = "") {
    std::string format_str = "{{throttle: {1:{0}},\n steer: {2:{0}},";
    format_str += "\n pitch: {3:{0}},\n yaw: {4:{0}},\n roll: {5:{0}},\n ";
    format_str += "boost: {6},\n jump: {7},\n powerslide: {8}}}";
    return str(format_str).attr("format")(spec, carControls.throttle, carControls.steer,
                carControls.pitch, carControls.yaw, carControls.roll,
                carControls.boost, carControls.jump, carControls.handbrake);
}

// CarState
Angle CarStateGetAngle(const CarState &car_state) {
    return Angle().FromRotMat(car_state.rotMat);
}

// Car
CarConfig CarGetConfig(const Car &car) {
    return car.config;
}

CarControls CarGetControls(const Car &car) {
    return car.controls;
}

void CarSetControls(Car &car, CarControls &carControls) {
    car.controls = carControls;
}

// Arena
Ball* ArenaGetBall(Arena &arena) {
    return arena.ball;
}

void ArenaSetGoalScoreCallback(Arena& self, object pyCallback) {
    GoalScoreEventFn callbackFn = [pyCallback](Arena* arena, Team scoringTeam, void*) {
        pyCallback(ptr(arena), scoringTeam);
    };
    self.SetGoalScoreCallback(callbackFn);
}

void ArenaWriteToFile(Arena &arena, std::string path_str) {
    const std::filesystem::path path = std::filesystem::u8path(path_str);
    arena.WriteToFile(path);
}

Arena* ArenaLoadFromFile(std::string path_str) {
    const std::filesystem::path path = std::filesystem::u8path(path_str);
    return Arena::LoadFromFile(path);
}

BOOST_PYTHON_MODULE(pyrocketsim) {

    Py_Initialize();
    np::initialize();

    def("init", &RocketSim::Init);

    class_<Vec>("Vec", init<float, float, float>((arg("x")=0, arg("y")=0, arg("z")=0)))
        .def_readwrite("x", &Vec::x)
        .def_readwrite("y", &Vec::y)
        .def_readwrite("z", &Vec::z)

        .def("is_zero", &Vec::IsZero)
        .def("length_sq", &Vec::LengthSq)
        .def("length", &Vec::Length)
        .def("length", &Vec::Length)
        .def("dot", &Vec::Dot, arg("other"))
        .def("cross", &Vec::Cross, arg("other"))
        .def("dist_sq", &Vec::DistSq, arg("other"))
        .def("dist", &Vec::Dist, arg("other"))
        .def("dist_sq_2d", &Vec::DistSq2D, arg("other"))
        .def("dist_2d", &Vec::Dist2D, arg("other"))
        .def("normalized", &Vec::Normalized)
        .def("__getitem__", &VecGetItem)
        .def("__setitem__", &VecSetItem)

        .def(self < self)
        .def(self > self)

        .def(self + self)
        .def(self - self)
        .def(self * self)
        .def(self / self)

        .def(self += self)
        .def(self -= self)
        .def(self *= self)
        .def(self /= self)

        // some operators were not defined in RocketSim at the time of writing this
        .def(self * float())
        .def(self / float())

        .def(self *= float())
        .def(self /= float())

        .def(-self)

        .def("as_tuple", &VecAsTuple)
        .def("as_numpy", &VecAsNumpy)

        .def("__format__", &VecFormat, arg("spec"))
        .def("__str__", &VecFormat, arg("spec")="")
        .def("__repr__", &VecFormat, arg("spec")="");
    
    class_<RotMat>("RotMat", init<>())
        .def(init<Vec, Vec, Vec>((arg("forward"), arg("right"), arg("up"))))

        .def("get_identity", &RotMat::GetIdentity)  // static

        .def("__getitem__", &RotMatGetItem)
        .def("__setitem__", &RotMatSetItem)

        .def(self + self)
        .def(self - self)

        .def(self * float())
        .def(self / float())

        .def(self *= float())
        .def(self /= float())

        .def("dot", &RotMat::Dot, arg("vec"))
        .def("transpose", &RotMat::Transpose)

        .def("as_numpy", &RotMatAsNumpy)

        .def("__format__", &RotMatFormat, arg("spec")="")
        .def("__str__", &RotMatFormat, arg("spec")="")
        .def("__repr__", &RotMatFormat, arg("spec")="");

    class_<Angle>("Angle",
        init<float, float, float>((arg("yaw")=0, arg("pitch")=0, arg("roll")=0)))

        .def_readwrite("yaw", &Angle::yaw)
        .def_readwrite("pitch", &Angle::pitch)
        .def_readwrite("roll", &Angle::roll)

        .def("from_rot_mat", &Angle::FromRotMat, arg("rot_mat"))  // static
        .def("to_rot_mat", &Angle::ToRotMat)
        .def("get_forward_vector", &Angle::GetForwardVector)
        .def("normalize_fix", &Angle::NormalizeFix)

        .def("as_tuple", &AngleAsTuple)
        .def("as_numpy", &AngleAsNumpy)

        .def("__format__", &AngleFormat, arg("spec")="")
        .def("__str__", &AngleFormat, arg("spec")="")
        .def("__repr__", &AngleFormat, arg("spec")="");

    class_<BoostPadState>("BoostPadState", init<>())
        .def_readwrite("is_active", &BoostPadState::isActive)
        .def_readwrite("cooldown", &BoostPadState::cooldown)
        .def("__format__", &BoostPadStateFormat, arg("spec")="")
        .def("__str__", &BoostPadStateFormat, arg("spec")="")
        .def("__repr__", &BoostPadStateFormat, arg("spec")="");

    class_<BoostPad>("BoostPad", no_init)
        .def_readonly("is_big", &BoostPad::isBig)
        .def_readonly("pos", &BoostPad::pos)
        .def("get_state", &BoostPad::GetState)
        .def("set_state", &BoostPad::SetState, arg("boost_pad_state"));

    register_ptr_to_python<BoostPad*>();

    class_<std::vector<BoostPad*> >("BoostPadVector")
        .def(vector_indexing_suite<std::vector<BoostPad*>>());

    class_<BallState>("BallState", init<>())
        .def_readwrite("pos", &BallState::pos)
        .def_readwrite("vel", &BallState::vel)
        .def_readwrite("ang_vel", &BallState::angVel);

    class_<Ball, boost::noncopyable>("Ball", no_init)
        .def("get_state", &Ball::GetState)
        .def("set_state", &Ball::SetState, arg("ball_state"))
        .def("get_radius", &BallGetRadius);

    register_ptr_to_python<Ball*>();

    class_<decltype(CarState::worldContact)>("WorldContact", no_init)
        .def_readwrite("has_contact", &decltype(CarState::worldContact)::hasContact)
        .def_readwrite("contact_normal", &decltype(CarState::worldContact)::contactNormal)
        .def("__format__", &WorldContactFormat, arg("spec")="")
        .def("__str__", &WorldContactFormat, arg("spec")="")
        .def("__repr__", &WorldContactFormat, arg("spec")="");

    class_<decltype(CarState::carContact)>("CarContact", no_init)
        .def_readwrite("other_car_id", &decltype(CarState::carContact)::otherCarID)
        .def_readwrite("cooldown_timer", &decltype(CarState::carContact)::cooldownTimer)
        .def("__format__", &CarContactFormat, arg("spec")="")
        .def("__str__", &CarContactFormat, arg("spec")="")
        .def("__repr__", &CarContactFormat, arg("spec")="");

    implicitly_convertible<CarControls, boost::python::object>();

    class_<CarControls>("CarControls")
        .def_readwrite("throttle", &CarControls::throttle)
        .def_readwrite("steer", &CarControls::steer)
        .def_readwrite("pitch", &CarControls::pitch)
        .def_readwrite("yaw", &CarControls::yaw)
        .def_readwrite("roll", &CarControls::roll)
        .def_readwrite("boost", &CarControls::boost)
        .def_readwrite("jump", &CarControls::jump)
        .def_readwrite("handbrake", &CarControls::handbrake)
        .def("clamp_fix", &CarControls::ClampFix)

        .def("__format__", &CarControlsFormat, arg("spec")="")
        .def("__str__", &CarControlsFormat, arg("spec")="")
        .def("__repr__", &CarControlsFormat, arg("spec")="");

    class_<CarState>("CarState")
        .def_readwrite("pos", &CarState::pos)
        .def_readwrite("rot_mat", &CarState::rotMat)
        .def_readwrite("vel", &CarState::vel)
        .def_readwrite("ang_vel", &CarState::angVel)

        .add_property("angles", &CarStateGetAngle)

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

    enum_<Team>("Team")
        .value("BLUE", Team::BLUE)
        .value("ORANGE", Team::ORANGE)
        .export_values();

    class_<WheelPairConfig>("WheelPairConfig")
        .def_readwrite("wheel_radius", &WheelPairConfig::wheelRadius)
        .def_readwrite("suspension_rest_length", &WheelPairConfig::suspensionRestLength)
        .def_readwrite("connection_point_offset", &WheelPairConfig::connectionPointOffset);

    class_<CarConfig>("CarConfig")
        .def_readwrite("hitbox_size", &CarConfig::hitboxSize)
        .def_readwrite("hitbox_pos_offset", &CarConfig::hitboxPosOffset)
        .def_readwrite("front_wheels", &CarConfig::frontWheels)
        .def_readwrite("back_wheels", &CarConfig::backWheels)
        .def_readwrite("dodge_deadzone", &CarConfig::dodgeDeadzone);

    scope().attr("OCTANE") = &CAR_CONFIG_OCTANE;
    scope().attr("DOMINUS") = &CAR_CONFIG_DOMINUS;
    scope().attr("PLANK") = &CAR_CONFIG_PLANK;
    scope().attr("BREAKOUT") = &CAR_CONFIG_BREAKOUT;
    scope().attr("HYBRID") = &CAR_CONFIG_HYBRID;
    scope().attr("MERC") = &CAR_CONFIG_MERC;

    class_<Car, boost::noncopyable>("Car", no_init)
        .def("get_config", &CarGetConfig)
        .def_readonly("team", &Car::team)
        .def_readonly("id", &Car::id)
        .def("get_controls", &CarGetControls)
        .def("set_controls", &CarSetControls)
        .def("get_state", &Car::GetState)
        .def("set_state", &Car::SetState, arg("car_state"))
        .def("demolish", &Car::Demolish)
        .def("respawn", &Car::Respawn, arg("seed")=-1)
        .add_property("forward_dir", &Car::GetForwardDir)
        .add_property("right_dir", &Car::GetRightDir)
        .add_property("up_dir", &Car::GetUpDir);

    register_ptr_to_python<Car*>();

    class_<std::vector<Car*> >("CarVector")
        .def(vector_indexing_suite<std::vector<Car*>>());

    enum_<GameMode>("GameMode")
        .value("SOCCAR", GameMode::SOCCAR)
        .export_values();

    class_<Arena, boost::noncopyable>("Arena", no_init)

        .def("__init__", make_constructor(&Arena::Create,
            default_call_policies(), (arg("game_mode")=GameMode::SOCCAR,
                arg("tick_rate")=120)))

        .def_readonly("game_mode", &Arena::gameMode)
        .def_readonly("tick_time", &Arena::tickTime)
        .add_property("tick_rate", &Arena::GetTickRate)
        .def_readonly("tick_count", &Arena::tickCount)
        .def_readonly("ball", &Arena::ball)

        .def("get_boost_pads", &Arena::GetBoostPads,
            return_value_policy<reference_existing_object>())

        .def("get_cars", &Arena::GetCars,
            return_value_policy<reference_existing_object>())

        .def("add_car", &Arena::AddCar, (arg("team"), arg("config")=CAR_CONFIG_OCTANE),
            return_value_policy<reference_existing_object>())

        .def("remove_car", &Arena::RemoveCar, arg("car"))
        .def("get_car_from_id", &Arena::RemoveCar, arg("id"))

        .def("set_goal_score_call_back", &ArenaSetGoalScoreCallback,
            (arg("self"), arg("callback_fn")))

        .def("write_to_file", &ArenaWriteToFile, arg("path_str"))
        .def("load_from_file", &ArenaLoadFromFile, arg("path_str"),
            return_value_policy<reference_existing_object>())

        .def("step", &Arena::Step)
        .def("clone", &Arena::Clone, arg("copy_callbacks"),
            return_value_policy<manage_new_object>())
        .def("reset_to_random_kickoff", &Arena::ResetToRandomKickoff, arg("seed")=-1);

    register_ptr_to_python<Arena*>();

}