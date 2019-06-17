cmake_minimum_required(VERSION 2.8.3)
project(auv_guidance)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(auv_navigation)
find_package(auv_control)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

# catkin_package(
#   INCLUDE_DIRS 
#     include 
#     ${EIGEN3_INCLUDE_DIR} 
#     ${CERES_INCLUDE_DIRS}
#   LIBRARIES ${PROJECT_NAME}_lib 
# )

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CERES_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    src/min_jerk_trajectory.cpp
    src/min_jerk_time_solver.cpp
    src/basic_trajectory.cpp
    src/simultaneous_trajectory.cpp
    src/long_trajectory.cpp
    src/tgen_limits.cpp
    src/waypoint.cpp
    #src/monotonic_trajectory.cpp
)

target_link_libraries(${PROJECT_NAME} SHARED
  ${auv_navigation_LIBRARIES}
  ${auv_control_LIBRARIES}
  ${catkin_LIBRARIES} 
  ${EIGEN3_LIBRARIES} 
  ${CERES_LIBRARIES}
)

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES})
add_dependencies(test_node ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include
        DESTINATION include/auv_gnc
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)