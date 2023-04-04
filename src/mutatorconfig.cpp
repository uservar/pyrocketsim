#include <pybind11/pybind11.h>

#include "../RocketSim/src/Sim/MutatorConfig/MutatorConfig.h"

namespace py = pybind11;
using namespace pybind11::literals;


void init_mutatorconfig(py::module_ &m) {

    py::enum_<DemoMode>(m, "DemoMode")
        .value("NORMAL", DemoMode::NORMAL)
        .value("ON_CONTACT", DemoMode::ON_CONTACT)
        .value("DISABLED", DemoMode::DISABLED);

    py::class_<MutatorConfig>(m, "MutatorConfig")
        .def(py::init<>())
        .def_readwrite("gravity", &MutatorConfig::gravity)
        .def_readwrite("car_mass", &MutatorConfig::carMass)
        .def_readwrite("car_world_friction", &MutatorConfig::carWorldFriction)
        .def_readwrite("car_world_restitution", &MutatorConfig::carWorldRestitution)
        .def_readwrite("ball_mass", &MutatorConfig::ballMass)
        .def_readwrite("ball_max_speed", &MutatorConfig::ballMaxSpeed)
        .def_readwrite("ball_drag", &MutatorConfig::ballDrag)
        .def_readwrite("ball_world_friction", &MutatorConfig::ballWorldFriction)
        .def_readwrite("ball_world_restitution", &MutatorConfig::ballWorldRestitution)
        .def_readwrite("jump_accel", &MutatorConfig::jumpAccel)
        .def_readwrite("jump_immediate_force", &MutatorConfig::jumpImmediateForce)
        .def_readwrite("boost_accel", &MutatorConfig::boostAccel)
        .def_readwrite("boost_used_per_second", &MutatorConfig::boostUsedPerSecond)
        .def_readwrite("respawn_delay", &MutatorConfig::respawnDelay)
        .def_readwrite("bump_cooldown_time", &MutatorConfig::bumpCooldownTime)
        .def_readwrite("boost_pad_cooldown_big", &MutatorConfig::boostPadCooldown_Big)
        .def_readwrite("boost_pad_cooldown_small", &MutatorConfig::boostPadCooldown_Small)
        .def_readwrite("car_spawn_boost_amount", &MutatorConfig::carSpawnBoostAmount)
        .def_readwrite("ball_hit_extra_force_scale", &MutatorConfig::ballHitExtraForceScale)
        .def_readwrite("bump_force_scale", &MutatorConfig::bumpForceScale)
        .def_readwrite("ball_radius", &MutatorConfig::ballRadius)
        .def_readwrite("demo_mode", &MutatorConfig::demoMode)
        .def_readwrite("enable_team_demos", &MutatorConfig::enableTeamDemos);
}