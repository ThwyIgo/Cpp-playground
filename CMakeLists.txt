cmake_minimum_required(VERSION 3.10)
project(Playground VERSION 1.0)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(bin src/main.cpp)
