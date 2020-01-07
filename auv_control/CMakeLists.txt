cmake_minimum_required(VERSION 2.8.3)
project(auv_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  auv_core
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(ct_optcon REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/LQRWeights.cfg
)

###########
## Build ##
###########

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
  INCLUDE_DIRS 
    include 
    ${EIGEN3_INCLUDE_DIR} 
    ${CERES_INCLUDE_DIRS}
    ${ct_optcon_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake 
  ${EIGEN3_INCLUDE_DIR} 
  ${CERES_INCLUDE_DIRS}
  ${ct_optcon_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    src/auv_lqr.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  ${EIGEN3_LIBRARIES} 
  ${CERES_LIBRARIES}
  ct_optcon
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)