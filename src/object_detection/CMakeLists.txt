cmake_minimum_required(VERSION 3.5.1)

catkin_add_env_hooks("setupvars" SHELLS bash DIRECTORY "${CMAKE_SOURCE_DIR}/../")

project(object_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-march=native -Wall -Ofast -fno-signed-zeros")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
set(OpenCV_DIR /opt/intel/openvino_2022/opencv/cmake)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS) # get rid of a warning


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_filters
  image_transport
  rviz
  roslaunch
  message_generation
  sensor_msgs
)


find_package(catkin REQUIRED)
find_package(realsense2 CONFIG REQUIRED)
find_package(OpenCV 4.6.0 EXACT REQUIRED)
find_package(Threads REQUIRED)
find_package(MLPACK REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenVINO REQUIRED)



add_message_files(
  FILES
  string_arr.msg
  detection.msg
  detection_arr.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${MLPACK_INCLUDE_DIRS}
  src/
)

add_executable(detect3d src/detect3d.cpp src/yolo.cpp src/util.cpp src/object.cpp src/QR.cpp src/tracker.cpp)

target_link_libraries(detect3d
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS} 
    Threads::Threads 
    zbar
    dbus-1
    realsense2::fw 
    realsense2::usb 
    realsense2::realsense2 
    realsense2::realsense-file
    OpenMP::OpenMP_CXX
)

add_dependencies(detect3d object_detection_generate_messages_cpp)