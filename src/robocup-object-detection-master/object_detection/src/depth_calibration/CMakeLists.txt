cmake_minimum_required(VERSION 3.5.1)
project(depth_calibration)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-march=native -fopenmp -Wall -Wno-unused-but-set-variable") # suppress a known warning. Check for all others
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS) # get rid of a warning
add_compile_definitions(_GLIBCXX_PARALLEL)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_filters
  image_transport
  roslaunch
  message_generation
  sensor_msgs
)


find_package(catkin REQUIRED)
find_package(realsense2 CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

message(
  "OPENCV VERSION ${OpenCV_VERSION}\n"
  "OPENCV DIR ${OpenCV_DIR}\n"
  "OPENCV LIBS ${OpenCV_LIBS}"
)

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
  ../common
)

add_executable(collect ./src/collect_raw_data.cpp ../common/yolo.cpp ../common/util.cpp)

target_link_libraries(collect
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS} 
    Threads::Threads 
    zbar
    dbus-1
    realsense2::fw 
    realsense2::usb 
    realsense2::realsense2 
    realsense2::realsense-file
)
add_dependencies(collect object_detection_generate_messages_cpp)