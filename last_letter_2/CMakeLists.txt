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
  dynamic_reconfigure
)
find_package(Boost REQUIRED)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/PD_gains.cfg
)

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

add_executable(joy2chan_node src/joy2chan_node.cpp) 
add_executable(hexacopter_contr_node src/controllers/hexacopter_contr_node.cpp) 
add_executable(quadcopter_contr_node src/controllers/quadcopter_contr_node.cpp) 
add_executable(plane_contr_node src/controllers/plane_contr_node.cpp) 
add_executable(core_node src/core_node.cpp)
add_executable(chan2srv_node src/chan2srv_node.cpp)

add_dependencies(joy2chan_node ${catkin_EXPORTED_TARGETS})
add_dependencies(hexacopter_contr_node ${catkin_EXPORTED_TARGETS})
add_dependencies(quadcopter_contr_node ${catkin_EXPORTED_TARGETS})
add_dependencies(plane_contr_node ${catkin_EXPORTED_TARGETS})
add_dependencies(core_node ${catkin_EXPORTED_TARGETS})
add_dependencies(chan2srv_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(joy2chan_node ${catkin_LIBRARIES})
target_link_libraries(hexacopter_contr_node ${catkin_LIBRARIES})
target_link_libraries(quadcopter_contr_node ${catkin_LIBRARIES})
target_link_libraries(plane_contr_node ${catkin_LIBRARIES})
target_link_libraries(core_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(chan2srv_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



