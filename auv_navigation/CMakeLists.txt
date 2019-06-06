cmake_minimum_required(VERSION 2.8.3)
project(auv_navigation)

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
)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(test_node src/test_node.cpp src/auv_math_lib.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})
add_dependencies(test_node ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib 
    src/auv_math_lib.cpp
    src/kalman_filter.cpp
    src/ekf_translation.cpp
)

target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_lib
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)