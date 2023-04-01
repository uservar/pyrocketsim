#include <string>
#include <filesystem>
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

        .def("get_boost_pads", &Arena::GetBoostPads)
        .def("get_cars", &Arena::GetCars)

        .def("add_car", &Arena::AddCar, "team"_a, "config"_a = CAR_CONFIG_OCTANE,
            py::return_value_policy::reference)

        .def("add_car", [](Arena &arena, Team team, uint32_t preset) {
            if (0 <= preset && preset < 7) {
                return arena.AddCar(team, *carConfigPresets[preset]);
            }
            else throw py::index_error("list index out of range");
        }, "team"_a, "config"_a = CAR_CONFIG_OCTANE,
            py::return_value_policy::reference)

        .def("remove_car", &Arena::RemoveCar, "car"_a)
        .def("get_car_from_id", &Arena::GetCarFromID, "id"_a)

        .def("set_goal_score_callback", &ArenaSetGoalScoreCallback)
        .def("set_goal_score_call_back", &ArenaSetGoalScoreCallback)  // I goofed

        .def("write_to_file", &Arena::WriteToFile, "path"_a)
        .def_static("load_from_file", &Arena::LoadFromFile)

        .def("step", &Arena::Step)
        .def("clone", &Arena::Clone, "copy_callbacks"_a)
        .def("reset_to_random_kickoff", &Arena::ResetToRandomKickoff, "seed"_a = -1);
}
