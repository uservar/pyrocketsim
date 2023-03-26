cmake_minimum_required(VERSION 3.15)

project(PyRocketSim)

add_subdirectory(RocketSim)

set(Python_USE_STATIC_LIBS ON)
find_package(Python COMPONENTS Development.Module REQUIRED)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS python)

include_directories(${Python_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${Python_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

message("${Boost_INCLUDE_DIRS}")
message("${Boost_LIBRARIES}")
message("${Boost_LIBRARY_DIRS}")
message("${Python_INCLUDE_DIRS}")
message("${Python_LIBRARIES}")
message("${Python_LIBRARY_DIRS}")

Python_add_library(pyrocketsim src/pyrocketsim.cpp)

target_compile_options(RocketSim PRIVATE -fPIC)
set_target_properties(pyrocketsim PROPERTIES CXX_STANDARD 20)
set_target_properties(pyrocketsim PROPERTIES PREFIX "")

target_link_libraries(pyrocketsim PRIVATE ${Boost_LIBRARIES} ${Python_LIBRARIES})
target_link_libraries(pyrocketsim PRIVATE RocketSim)
