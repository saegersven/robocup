cmake_minimum_required(VERSION 3.10)

project(robocup VERSION 1.0)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(robocup src/main.cpp)

target_link_libraries(robocup ${OpenCV_LIBS})