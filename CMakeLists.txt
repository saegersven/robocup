cmake_minimum_required(VERSION 3.16)
project(robocup VERSION 1.0)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory("libs/tensorflow/lite"
	"libs/tensorflow-lite" EXCLUDE_FROM_ALL)

add_executable(robocup src/main.cc src/robot.cc src/utils.cc src/vision.cc src/line.cc src/green_model.cc)
target_link_libraries(robocup ${OpenCV_LIBS})
target_link_libraries(robocup tensorflow-lite)