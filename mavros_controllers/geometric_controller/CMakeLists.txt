cmake_minimum_required(VERSION 2.8.12)
project(geometric_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  tf
  mavros
  mavros_extras
  mavros_msgs
  mavlink
)
# find_package(BLAS)
# find_package(LAPACK)

generate_dynamic_reconfigure_options(
  cfg/GeometricController.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geometric_controller
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs sensor_msgs tf
)

#############
# LIBRARIES #
#############

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}

)

add_library(${PROJECT_NAME}
  src/geometric_controller.cpp 

)
add_dependencies(geometric_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
#add_executable(test_
 # src/test_.cpp
#)
#target_link_libraries(test_ ${PROJECT_NAME} ${catkin_LIBRARIES} liblapacke.a liblapack.a #libblas.a libcblas.a librefblas.a libm.a libgfortran.a libquadmath.so)



add_executable(geometric_controller_node
  src/geometric_controller_node.cpp
)
add_dependencies(geometric_controller_node geometric_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(geometric_controller_node ${PROJECT_NAME} ${catkin_LIBRARIES})
##########
# EXPORT #
##########
# cs_install()
# cs_export()
#############
## Testing ##
#############


