cmake_minimum_required(VERSION 3.16)

project(PyRoboCppInterface LANGUAGES CXX)

add_library(pyrobo_contestant OBJECT
    src/contestant.cpp
)

target_include_directories(pyrobo_contestant
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(pyrobo_contestant PUBLIC cxx_std_17)
