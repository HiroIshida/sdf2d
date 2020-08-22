cmake_minimum_required( VERSION 2.8 FATAL_ERROR)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -Wall -fPIC")

set(nlohmann_json_INCLUDE_DIRS json/include)
include_directories(${nlohmann_json_INCLUDE_DIRS})
add_executable(out main.cpp)

option(BUILD_PYTHON_INTERFACE "Set when you want to build python interface" ON)
if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(pybind11)
  pybind11_add_module(sdf2d py_api.cpp)
endif(BUILD_PYTHON_INTERFACE)
