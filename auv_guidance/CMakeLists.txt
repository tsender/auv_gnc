cmake_minimum_required(VERSION 2.8.3)
project(auv_guidance)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  auv_navigation
  auv_control
)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

catkin_package(
  INCLUDE_DIRS 
    include 
    ${EIGEN3_INCLUDE_DIR} 
    ${CERES_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR} 
  ${CERES_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    src/min_jerk_trajectory.cpp
    src/min_jerk_time_solver.cpp
    src/basic_trajectory.cpp
    src/simultaneous_trajectory.cpp
    src/long_trajectory.cpp
    src/tgen_limits.cpp
    src/waypoint.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  ${EIGEN3_LIBRARIES} 
  ${CERES_LIBRARIES}
)

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES})
add_dependencies(test_node ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS test_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)