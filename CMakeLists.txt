cmake_minimum_required(VERSION 3.16)
project(robocup VERSION 1.0)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(WiringPi REQUIRED)
include_directories(${WIRINGPI_INCLUDE_DIRS})

find_package(Threads REQUIRED) 

set(ENABLE_ML)
add_subdirectory("libs/tensorflow/tensorflow/lite"
#	"libs/tflite_build/tensorflow-lite" EXCLUDE_FROM_ALL)

include_directories("libs/tensorflow")
include_directories("libs/tflite_build/flatbuffers/include")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories("${EIGEN3_INCLUDE_DIR}")
include_directories("libs/json/include")
include_directories("libs/frugally-deep/include")
include_directories("libs/FunctionalPlus/include")

add_executable(robocup src/main.cc src/robot.cc src/utils.cc src/vision.cc src/line.cc src/rescue.cc src/silver_ml.cc)

target_link_libraries(robocup atomic)

target_link_libraries(robocup ${OpenCV_LIBS})
# target_link_libraries(robocup tensorflow-lite)

target_link_libraries(robocup ${WIRINGPI_LIBRARIES})
target_link_libraries(robocup ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(robocup i2c)
target_link_libraries(robocup Eigen3::Eigen)