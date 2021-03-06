cmake_minimum_required(VERSION 2.8.3)
project(auv_gnc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  auv_core  
  auv_guidance
  auv_navigation
  auv_control
  auv_msgs
  roscpp
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

catkin_package(
   CATKIN_DEPENDS roscpp dynamic_reconfigure  
   INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
)

include_directories(include 
   ${catkin_INCLUDE_DIRS} 
   ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake 
   ${EIGEN3_INCLUDE_DIRS}
)

add_executable(trans_ekf src/trans_ekf_node.cpp src/trans_ekf.cpp)
target_link_libraries(trans_ekf ${catkin_LIBRARIES})
add_dependencies(trans_ekf ${catkin_EXPORTED_TARGETS})

add_executable(guidance_controller src/guidance_controller_node.cpp src/guidance_controller.cpp)
target_link_libraries(guidance_controller ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(guidance_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS trans_ekf guidance_controller
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)