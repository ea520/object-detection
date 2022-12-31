cmake_minimum_required(VERSION 3.25)

project(object_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_filters
  rviz
  roslaunch
  sensor_msgs
)

find_package(realsense2 CONFIG REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS timer REQUIRED)


include(GetOpenCV.cmake)
include(GetZbar.cmake)
catkin_package()

file(GLOB SRC_FILES "src/*.cpp")

add_executable(detect3d  ${SRC_FILES})
add_dependencies(detect3d opencv)
add_dependencies(detect3d zbar_mchehab)

target_include_directories(detect3d PRIVATE ${OPENCV_DIR}/include/)
target_include_directories(detect3d PRIVATE ${catkin_INCLUDE_DIRS})

target_link_directories(detect3d PRIVATE ${OPENCV_DIR}/lib/)
target_link_directories(detect3d PRIVATE ${ZBAR_DIR}/lib/)

target_link_libraries(
    detect3d
    ${OpenCV_LIB_COMPONENTS}
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
    Threads::Threads 
    zbar
    realsense2::realsense2 
)
target_compile_options(
  detect3d
  PRIVATE
  "-march=native"
)