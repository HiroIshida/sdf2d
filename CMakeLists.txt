cmake_minimum_required( VERSION 2.8 FATAL_ERROR)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -Wall -fPIC")

set(nlohmann_json_INCLUDE_DIRS json/include)
include_directories(${nlohmann_json_INCLUDE_DIRS})

add_executable(out main.cpp)
