cmake_minimum_required(VERSION 3.13)

project(PyRocketSim)

set(CMAKE_SHARED_LIBRARY_PREFIX "")

add_subdirectory(RocketSim)

if(DEFINED PYTHON_LIBS_VERSION)
    find_package(PythonLibs ${PYTHON_LIBS_VERSION} EXACT REQUIRED)
else()
    find_package(PythonLibs REQUIRED)
endif()

find_package(Boost REQUIRED COMPONENTS python numpy)

include_directories(${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(pyrocketsim SHARED src/pyrocketsim.cpp)

target_link_libraries(pyrocketsim PRIVATE ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
target_link_libraries(pyrocketsim PRIVATE RocketSim)

target_compile_options(RocketSim PRIVATE -fPIC)
set_target_properties(pyrocketsim PROPERTIES CXX_STANDARD 20)
