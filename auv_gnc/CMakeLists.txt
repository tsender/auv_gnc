cmake_minimum_required(VERSION 2.8.3)
project(auv_gnc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen3)

###########
## Build ##
###########

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS ${Eigen3_LIBRARIES} 
)
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(test_node src/test_node.cpp src/auv_math_lib.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})
add_dependencies(test_node ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib 
    src/auv_math_lib.cpp
    src/auv_model.cpp
    src/kalman_filter.cpp
    src/ekf_translation.cpp
    src/distance_motion_planner.cpp
    src/trajectory_line.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC include ${Eigen3_INCLUDE_DIRS})

