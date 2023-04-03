#include <pybind11/pybind11.h>

#include "../RocketSim/src/Sim/Car/Car.h"
#include "../RocketSim/src/RLConst.h"

namespace py = pybind11;
using namespace pybind11::literals;


void init_car(py::module_ &m) {

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
        .def_readwrite("ball_hit_info", &CarState::ballHitInfo)
        
        .def_readwrite("last_controls", &CarState::lastControls);

    py::enum_<Team>(m, "Team")
        .value("BLUE", Team::BLUE)
        .value("ORANGE", Team::ORANGE)
        .export_values();

    py::class_<Car>(m, "Car")
        .def("get_config", [](const Car &car) {return car.config;})
        .def_readonly("team", &Car::team)
        .def_readonly("id", &Car::id)
        .def("get_controls", [](const Car &car) {return car.controls;})
        .def("set_controls", [](Car &car, CarControls &controls) {
            car.controls = controls; }, "car_controls"_a)
        .def("get_state", &Car::GetState)
        .def("set_state", &Car::SetState, "car_state"_a)
        .def("demolish", &Car::Demolish, "respawn_delay"_a = RLConst::BOOST_SPAWN_AMOUNT)
        .def("respawn", &Car::Respawn, "seed"_a = -1, "boost_amount"_a = RLConst::BOOST_SPAWN_AMOUNT)
        .def_property_readonly("forward_dir", &Car::GetForwardDir)
        .def_property_readonly("right_dir", &Car::GetRightDir)
        .def_property_readonly("up_dir", &Car::GetUpDir);

    py::class_<std::unordered_set<Car*>>(m, "CarVector")

        .def("__len__", [](const std::unordered_set<Car*> &st) {
            return st.size();
        })

        .def("__getitem__", [](const std::unordered_set<Car*> &st, uint32_t index) {
            if (0 <= index && index < st.size()) {
                std::vector<Car*> vec(st.begin(), st.end());
                return vec[index];
            }
            throw py::index_error("list index out of range");
        }, py::return_value_policy::reference);

}