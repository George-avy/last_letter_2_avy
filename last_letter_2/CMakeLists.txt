cmake_minimum_required(VERSION 3.10.2)
project(last_letter_2)

find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  geometry_msgs
  rosgraph_msgs
  sensor_msgs
  std_msgs
  last_letter_2_msgs
  last_letter_2_libs
  tf2_ros
  tf2_geometry_msgs
)
find_package(Boost REQUIRED)

catkin_package(DEPENDS
  CATKIN_DEPENDS 
  std_msgs
 # INCLUDE_DIRS include
)

include_directories(
  include
 # include/propulsion
 ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIR}
 )

add_executable(joy2chan src/joy2chan.cpp) 
add_executable(controller src/controller.cpp) 
add_executable(core src/core.cpp)
add_executable(buttonFunctions src/buttonFunctions.cpp)

add_dependencies(joy2chan ${catkin_EXPORTED_TARGETS})
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
add_dependencies(core ${catkin_EXPORTED_TARGETS})
add_dependencies(buttonFunctions ${catkin_EXPORTED_TARGETS})

target_link_libraries(joy2chan ${catkin_LIBRARIES})
target_link_libraries(controller ${catkin_LIBRARIES})
target_link_libraries(core ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(buttonFunctions ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



