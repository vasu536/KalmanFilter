cmake_minimum_required(VERSION 3.8)
project(KF)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp Matrix.h Matrix.cpp)
add_executable(KF ${SOURCE_FILES})