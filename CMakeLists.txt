cmake_minimum_required(VERSION 2.8.3)
project(gpg)
set(CMAKE_BUILD_TYPE Release)

#########################
# ROS

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gpg
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/gpg.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/gpg_node.cpp)

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

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gpg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#################



# Eigen library
include_directories(${EIGEN3_INCLUDE_DIR})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message("------------------ PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")

## Set compiler optimization flags
set(CMAKE_CXX_FLAGS "-O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare")
# set(CMAKE_CXX_FLAGS "-O3 -fopenmp -march=native -mfpmath=sse -funroll-loops -fPIC -Wno-deprecated -Wenum-compare") # no improvement
# set(CMAKE_CXX_FLAGS "-frename-registers -Ofast -march=native -fopenmp -fPIC -Wno-deprecated -Wenum-compare") # no improvement
 
# Add the headers from the <include> directory
include_directories(include)
 
# Generate the shared library from the sources
add_library(${PROJECT_NAME}_grasp_candidates_generator SHARED src/${PROJECT_NAME}/candidates_generator.cpp)

# Other libraries
add_library(${PROJECT_NAME}_antipodal src/${PROJECT_NAME}/antipodal.cpp)
add_library(${PROJECT_NAME}_cloud_camera src/${PROJECT_NAME}/cloud_camera.cpp)
add_library(${PROJECT_NAME}_config_file src/${PROJECT_NAME}/config_file.cpp)
add_library(${PROJECT_NAME}_eigen_utils src/${PROJECT_NAME}/eigen_utils.cpp)
add_library(${PROJECT_NAME}_finger_hand src/${PROJECT_NAME}/finger_hand.cpp)
add_library(${PROJECT_NAME}_frame_estimator src/${PROJECT_NAME}/frame_estimator.cpp)
add_library(${PROJECT_NAME}_grasp src/${PROJECT_NAME}/grasp.cpp)
add_library(${PROJECT_NAME}_grasp_set src/${PROJECT_NAME}/grasp_set.cpp)
add_library(${PROJECT_NAME}_hand_search src/${PROJECT_NAME}/hand_search.cpp)
add_library(${PROJECT_NAME}_local_frame src/${PROJECT_NAME}/local_frame.cpp)
add_library(${PROJECT_NAME}_plot src/${PROJECT_NAME}/plot.cpp)
add_library(${PROJECT_NAME}_point_list src/${PROJECT_NAME}/point_list.cpp)

# This executable is for testing the shared library
add_executable(${PROJECT_NAME}_generate_candidates src/generate_candidates.cpp)
target_link_libraries(${PROJECT_NAME}_generate_candidates 
                      ${PROJECT_NAME}_config_file 
                      ${PROJECT_NAME}_grasp_candidates_generator)

# Linking for libraries
target_link_libraries(${PROJECT_NAME}_antipodal 
                      ${PROJECT_NAME}_point_list)

target_link_libraries(${PROJECT_NAME}_cloud_camera 
                      ${PROJECT_NAME}_eigen_utils 
                      ${PCL_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_eigen_utils 
                      ${EIGEN_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_frame_estimator 
                      ${PROJECT_NAME}_cloud_camera 
                      ${PROJECT_NAME}_local_frame)

target_link_libraries(${PROJECT_NAME}_grasp 
                      ${PROJECT_NAME}_finger_hand)

target_link_libraries(${PROJECT_NAME}_grasp_set 
                      ${PROJECT_NAME}_antipodal 
                      ${PROJECT_NAME}_grasp 
                      ${PROJECT_NAME}_local_frame 
                      ${PROJECT_NAME}_point_list)

target_link_libraries(${PROJECT_NAME}_hand_search 
                      ${PROJECT_NAME}_antipodal 
                      ${PROJECT_NAME}_cloud_camera  
                      ${PROJECT_NAME}_frame_estimator 
                      ${PROJECT_NAME}_grasp_set 
                      ${PROJECT_NAME}_plot)

target_link_libraries(${PROJECT_NAME}_local_frame 
                      ${PCL_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_plot 
                      ${PROJECT_NAME}_cloud_camera 
                      ${PROJECT_NAME}_grasp_set 
                      ${PROJECT_NAME}_local_frame)

target_link_libraries(${PROJECT_NAME}_grasp_candidates_generator 
                      ${PROJECT_NAME}_hand_search)
                      
target_link_libraries(${PROJECT_NAME}_point_list 
                      ${PROJECT_NAME}_eigen_utils)


# Rename executable.
set_target_properties(${PROJECT_NAME}_generate_candidates 
                      PROPERTIES OUTPUT_NAME generate_candidates 
                      PREFIX "")

set_target_properties(${PROJECT_NAME}_grasp_candidates_generator 
                      PROPERTIES OUTPUT_NAME grasp_candidates_generator)





# install some things with ROS
# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_grasp_candidates_generator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Set the location for library installation (/usr/lib in this case is not really necessary)
# Use "sudo make install" to apply
install(TARGETS ${PROJECT_NAME}_grasp_candidates_generator 
        DESTINATION /usr/lib)

install(DIRECTORY include/gpg DESTINATION /usr/local/include)
