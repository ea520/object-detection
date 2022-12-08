cmake_minimum_required(VERSION 3.5.1)

catkin_add_env_hooks("setupvars" SHELLS bash DIRECTORY "${CMAKE_SOURCE_DIR}/../")

project(object_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-march=native -Wall")
set(OpenCV_DIR /opt/intel/openvino_2022/opencv/cmake)


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
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS timer REQUIRED)


catkin_package(
    CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
file(GLOB SRC_FILES "src/*.cpp")

add_executable(detect3d  ${SRC_FILES})

target_link_libraries(detect3d
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS} 
    ${Boost_LIBRARIES}
    Threads::Threads 
    zbar
    realsense2::realsense2 
)
