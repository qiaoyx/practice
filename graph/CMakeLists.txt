cmake_minimum_required(VERSION 3.2)

project(tsp)

set(CMAKE_VERBOSE_MAKEFILE on)
set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")

set(CMAKE_CXX_FLAGS "-I. -lboost_graph -lboost_filesystem")
set(CMAKE_CXX_FLAGS_DEBUG   "-g3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(SRC_LIST tsp.cpp main.cpp)
add_executable(tsp ${SRC_LIST})
