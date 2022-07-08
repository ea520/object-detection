cmake_minimum_required(VERSION 3.5.1)
project(object_detection)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-march=native -Wall -Wno-unused-but-set-variable") # suppress a known warning. Check for all others
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
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
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(MLPACK REQUIRED)

message(
  "OPENCV DIR ${OpenCV_DIR}\n"
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
  ${MLPACK_INCLUDE_DIRS}
  src/common/
)

add_executable(.detect3d src/object_detection/detect3d.cpp src/common/yolo.cpp src/common/util.cpp)

target_link_libraries(.detect3d
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

add_dependencies(.detect3d object_detection_generate_messages_cpp)



add_executable(.detect2d src/object_detection/detect2d.cpp src/common/yolo.cpp)

target_link_libraries(.detect2d
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS} 
    Threads::Threads 
    zbar
    dbus-1
)
add_dependencies(.detect2d object_detection_generate_messages_cpp)

# the cluster executable
add_executable(.cluster src/object_detection/cluster.cpp)

target_link_libraries(.cluster 
    ${catkin_LIBRARIES} 
    Threads::Threads 
    ${MLPACK_LIBRARY}
)
add_dependencies(.cluster object_detection_generate_messages_cpp)



add_executable(.print_csv src/object_detection/print_csv.cpp)

target_link_libraries(.print_csv 
    ${catkin_LIBRARIES}
    Threads::Threads 
)
add_dependencies(.print_csv object_detection_generate_messages_cpp)


add_executable(.collect src/depth_calibration/collect_raw_data.cpp src/common/yolo.cpp src/common/util.cpp)

target_link_libraries(.collect
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
add_dependencies(.collect object_detection_generate_messages_cpp)
