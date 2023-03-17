cmake_minimum_required(VERSION 3.13)

project(PyRocketSim)

add_subdirectory(RocketSim)
add_subdirectory(pybind11)

pybind11_add_module(pyrocketsim MODULE src/pyrocketsim.cpp)

target_link_libraries(pyrocketsim PRIVATE RocketSim)

target_compile_options(RocketSim PRIVATE -fPIC)

set_target_properties(pyrocketsim PROPERTIES CXX_STANDARD 20)
