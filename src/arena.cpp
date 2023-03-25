#include <string>
#include <filesystem>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>

#include "../RocketSim/src/Sim/Arena/Arena.h"

namespace py = pybind11;
using namespace pybind11::literals;


void init_arena(py::module_ &m) {


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