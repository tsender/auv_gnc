cmake_minimum_required(VERSION 2.8.3)
project(auv_guidance)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  auv_navigation
  auv_control
)

find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

catkin_package(
  INCLUDE_DIRS include
)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME}_lib
    src/min_jerk_trajectory.cpp
    src/short_range_trajectory.cpp
    src/long_range_trajectory.cpp
    src/waypoint.cpp
)

target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_lib
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)