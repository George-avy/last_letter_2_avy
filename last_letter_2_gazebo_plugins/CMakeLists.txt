cmake_minimum_required(VERSION 3.10.2)
project(last_letter_2_gazebo_plugins)


find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  last_letter_2_msgs
  tf2_ros
  kdl_parser
)
find_package(Boost REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(DEPENDS
  CATKIN_DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

link_directories(/opt/ros/melodic/lib/)
set(MY_ADDED_LIBS   orocos-kdl;kdl_parser;urdf)

add_library(model_plugin src/model_plugin.cpp)
add_library(world_plugin src/world_plugin.cpp)

add_dependencies(model_plugin ${catkin_EXPORTED_TARGETS})
add_dependencies(world_plugin ${catkin_EXPORTED_TARGETS})

target_link_libraries(model_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${MY_ADDED_LIBS})
target_link_libraries(world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})