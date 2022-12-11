cmake_minimum_required(VERSION 3.5.1)

project(object_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
find_package(Threads REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS timer REQUIRED)




include(GetOpenCV.txt)
catkin_package(
)

file(GLOB SRC_FILES "src/*.cpp")

add_executable(detect3d  ${SRC_FILES})
add_dependencies(detect3d opencv46)
target_include_directories(detect3d PRIVATE ${OPENCV_DIR}/include/ ${catkin_INCLUDE_DIRS})
target_link_directories(detect3d PRIVATE ${OPENCV_DIR}/lib/)

target_link_libraries(detect3d
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS} 
    ${Boost_LIBRARIES}
    Threads::Threads 
    zbar
    realsense2::realsense2 
    ${OpenCV_LIB_COMPONENTS}
)
