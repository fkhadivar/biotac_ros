cmake_minimum_required(VERSION 2.8.3)
project(biotac_ros)
add_compile_options(-std=c++11)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)
find_package(Eigen3 REQUIRED eigen3) 

add_message_files(
  FILES
  fingerMsg.msg
)

add_service_files(
  FILES
  nnmodel.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES biotac_ros
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_generation
#  DEPENDS system_lib
)


include_directories( include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(biotc_sensor src/node_biotac_sensor.cpp src/sensor_process.cpp)
target_include_directories(biotc_sensor PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
target_link_libraries(biotc_sensor ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} )
add_dependencies(biotc_sensor ${PROJECT_NAME}_gencpp)

