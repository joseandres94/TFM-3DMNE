cmake_minimum_required(VERSION 3.0.2)
project(node_zed)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp nodelet message_generation sensor_msgs tf2_ros tf2_geometry_msgs)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED)
find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)
find_package(ZED 2 REQUIRED)
SET(SPECIAL_OS_LIBS "pthread")


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
#add_service_files(
#  DIRECTORY services/srv 
#  FILES
#  start_remote_stream.srv
#  stop_remote_stream.srv
#)
add_service_files(
  FILES
  start_remote_stream.srv
  stop_remote_stream.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nodes
#  CATKIN_DEPENDS zed_interfaces
#  DEPENDS system_lib
CATKIN_DEPENDS
sensor_msgs#
tf2_ros
tf2_geometry_msgs
message_generation
message_runtime
)

###############################################################################
# SOURCES

#set(TOOLS_SRC /home/isa/catkin_ws/src/zed-ros-wrapper/zed_nodelets/src/tools/src/sl_tools.cpp)#
set(ZED_NODELET_SRC /root/catkin_ws/src/node_zed/node_zed_nodelet/node_zed_nodelet.cpp)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
 ${CUDA_INCLUDE_DIRS}
 ${ZED_INCLUDE_DIRS}
 /root/catkin_ws/devel/include
 /opt/ros/melodic/include#
 /root/catkin_ws/src/node_zed
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})


SET(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES}
)#

add_library(node_zed_nodelet
    ${TOOLS_SRC}
    ${ZED_NODELET_SRC}
)#

#target_include_directories(ZEDNodelets PRIVATE ${INCLUDE_DIRS})
target_link_libraries(node_zed_nodelet ${LINK_LIBRARIES})#
add_dependencies(
	node_zed_nodelet
	${catkin_EXPORTED_TARGETS}
##    ${PROJECT_NAME}_gencfg
)#

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/nodes.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/nodes_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#add_executable(prueba prueba.cpp)
#target_link_libraries(prueba ${catkin_LIBRARIES})
#target_link_libraries(prueba ${PCL_LIBRARIES})
#target_link_libraries(prueba ${Boost_LIBRARIES})

#add_executable(prueba2 prueba2.cpp)
#target_link_libraries(prueba2 ${catkin_LIBRARIES})
#target_link_libraries(prueba2 ${PCL_LIBRARIES})
#target_link_libraries(prueba2 ${Boost_LIBRARIES})

#add_executable(prueba3 prueba3.cpp)
#target_link_libraries(prueba3 ${catkin_LIBRARIES})
#target_link_libraries(prueba3 ${PCL_LIBRARIES})
#target_link_libraries(prueba3 ${Boost_LIBRARIES})

#add_executable(normal normal.cpp)
#target_link_libraries(normal ${catkin_LIBRARIES})
#target_link_libraries(normal ${PCL_LIBRARIES})
#target_link_libraries(normal ${Boost_LIBRARIES})

#add_executable(mls mls.cpp)
#target_link_libraries(mls ${catkin_LIBRARIES})
#target_link_libraries(mls ${PCL_LIBRARIES})
#target_link_libraries(mls ${Boost_LIBRARIES})

#add_executable(prueba4 prueba4.cpp)
#target_link_libraries(prueba4 ${catkin_LIBRARIES})
#target_link_libraries(prueba4 ${PCL_LIBRARIES})
#target_link_libraries(prueba4 ${Boost_LIBRARIES})

#add_executable(imu_ned_to_enu imu_ned_to_enu.cpp)
#target_link_libraries(imu_ned_to_enu ${catkin_LIBRARIES})

ADD_EXECUTABLE(node_zed node_zed.cpp)
TARGET_LINK_LIBRARIES(node_zed
			${catkin_LIBRARIES}
			${ZED_LIBRARIES}
      ${SPECIAL_OS_LIBS}
      ${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY})
			#${TOOLS_SRC})
			#${LINK_LIBRARIES})


#add_executable(listener_cloudmap2 listener_cloudmap2.cpp)
#target_link_libraries(listener_cloudmap2 ${catkin_LIBRARIES})
#target_link_libraries(listener_cloudmap2 ${PCL_LIBRARIES})
#target_link_libraries(listener_cloudmap2 ${Boost_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(TARGETS 
  node_zed_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
  node_zed_nodelet_plugins.xml
#   # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nodes.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
