cmake_minimum_required(VERSION 2.8.3)
project(templatematching)

set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED COMPONENTS tf pcl_ros pcl_conversions roscpp rospy std_msgs genmsg sensor_msgs geometry_msgs jsk_recognition_msgs message_filters)

find_package(gazebo 9 REQUIRED sensor_msgs dynamic_reconfigure)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  jsk_recognition_msgs)


find_package(PCL REQUIRED)
find_package(PCL 1.7 REQUIRED)
set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.7/")
include_directories(${PCL_INCLUDE_DIRS})
include_directories("/usr/include/eigen3/")
include_directories("/usr/include/vtk-5.8/")
include_directories(include)
include_directories("/opt/ros/kinetic/include/")


generate_dynamic_reconfigure_options(
  cfg/matchingparam.cfg
  )



include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (chargematch src/correspondence_grouping.cpp)
target_link_libraries(chargematch ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(chargematch ${PROJECT_NAME}_gencfg)


catkin_package(
 CATKIN_DEPENDS gazebo_ros
)
