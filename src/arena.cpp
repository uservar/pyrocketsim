#include <string>
#include <filesystem>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl/filesystem.h>

#include "../RocketSim/src/Sim/Arena/Arena.h"

namespace py = pybind11;
using namespace pybind11::literals;


void ArenaSetGoalScoreCallback(Arena& arena, py::function callback_fn) {
    GoalScoreEventFn callback = [callback_fn](Arena* arena,
        Team scoringTeam, void* userInfo) {
        callback_fn(arena, scoringTeam);  // not using userInfo
    };
    arena.SetGoalScoreCallback(callback);
}

void ArenaSetCarBumpCallback(Arena& arena, py::function callback_fn) {
    CarBumpEventFn callback = [callback_fn](Arena* arena, Car* bumper,
        Car* victim, bool is_demo, void* userInfo) {
        callback_fn(arena, bumper, victim, is_demo);
    };
    arena.SetCarBumpCallback(callback);
}


void init_arena(py::module_ &m) {

    py::enum_<GameMode>(m, "GameMode")
        .value("SOCCAR", GameMode::SOCCAR)
        .value("THE_VOID", GameMode::THE_VOID)
        .export_values();

    py::class_<Arena>(m, "Arena")
        .def(py::init(&Arena::Create), "game_mode"_a = GameMode::SOCCAR, "tick_rate"_a = 120)
        .def_readonly("game_mode", &Arena::gameMode)

        .def("get_mutator_config", &Arena::GetMutatorConfig)
        .def("set_mutator_config", &Arena::SetMutatorConfig, "mutator_config"_a)

        .def_readonly("tick_time", &Arena::tickTime)
        .def_property_readonly("tick_rate", &Arena::GetTickRate)
        .def_readonly("tick_count", &Arena::tickCount)
        .def_readonly("ball", &Arena::ball)

        .def("get_boost_pads", &Arena::GetBoostPads, py::return_value_policy::reference)
        .def("get_cars", [](Arena &arena) {
            std::vector<Car*> sortedVec(arena._cars.begin(), arena._cars.end());
            std::sort(sortedVec.begin(), sortedVec.end(),
                [](Car* self, Car* other){ return self->id < other->id; }
            );
            return sortedVec;

        }, py::return_value_policy::reference)

        .def("add_car", &Arena::AddCar, "team"_a, "config"_a = CAR_CONFIG_OCTANE,
            py::return_value_policy::reference)

        .def("add_car", [](Arena &arena, Team team, uint8_t preset) {
           // overload for add_car which can take an enum/number for car preset
            return arena.AddCar(team, GetCarConfigFromPreset(preset));
        }, "team"_a, "preset"_a = CarPreset::OCTANE, py::return_value_policy::reference)

        .def("remove_car", py::overload_cast<uint32_t>(&Arena::RemoveCar), "id"_a)
        .def("remove_car", py::overload_cast<Car*>(&Arena::RemoveCar), "car"_a)

        .def("get_car", &Arena::GetCar, "id"_a, py::return_value_policy::reference)
        .def("get_car_from_id", &Arena::GetCar, "id"_a, py::return_value_policy::reference)

        .def("set_goal_score_callback", &ArenaSetGoalScoreCallback)
        .def("set_car_bump_callback", &ArenaSetCarBumpCallback)

        .def("write_to_file", &Arena::WriteToFile, "path"_a)
        .def_static("load_from_file", &Arena::LoadFromFile)

        .def("step", &Arena::Step)
        .def("clone", &Arena::Clone, "copy_callbacks"_a)
        .def("reset_to_random_kickoff", &Arena::ResetToRandomKickoff, "seed"_a = -1)
        .def("is_probably_going_in", &Arena::IsBallProbablyGoingIn, "max_time"_a = 2.f);
}
