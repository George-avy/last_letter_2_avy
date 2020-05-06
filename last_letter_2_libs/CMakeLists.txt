cmake_minimum_required(VERSION 3.10.2)
project(last_letter_2_libs)


find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES math_lib
  LIBRARIES geo_mag_declination
)

include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR}
)

add_library(math_lib src/math_lib.cpp)
add_library(math_lib src/environment.cpp)

target_link_libraries(math_lib ${catkin_LIBRARIES})
target_link_libraries(geo_mag_declination ${catkin_LIBRARIES})

install(TARGETS math_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS geo_mag_declination
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN ".h" PATTERN ".hpp"
  PATTERN ".svn" EXCLUDE)