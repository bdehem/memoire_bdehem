cmake_minimum_required(VERSION 2.8.3)
project(boris_drone)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  roscpp
  cv_bridge
  std_msgs
  image_transport
  sensor_msgs
  nav_msgs
  message_generation
  pcl_conversions
  pcl_ros
)

find_package( OpenCV REQUIRED core nonfree )# ocl gpu)

find_package( Ceres REQUIRED )

set(CMAKE_CXX_FLAGS "-Wno-deprecated -std=c++11 ${CMAKE_CXX_FLAGS}")
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
add_message_files(
  FILES
  BenchmarkInfoMsg.msg
  Pose3D.msg
  TargetDetected.msg
  PoseRef.msg
  StrategyMsg.msg
  DroneRole.msg
  DroneRoles.msg
  KeyPoint.msg
  ProcessedImageMsg.msg
  cellUpdate.msg
  BundleMsg.msg
  ObservationMsg.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#  FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs  # Or other packages containing msgs
 geometry_msgs
 sensor_msgs
 ardrone_autonomy
)

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
#  LIBRARIES boris_drone
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
CATKIN_DEPENDS ardrone_autonomy message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
	)


## Declare a C++ library
# add_library(boris_drone
#   src/${PROJECT_NAME}/boris_drone.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(boris_drone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(MAPPING_SOURCE_FILES
	src/map/mapping_node.cpp
	src/map/map.cpp
	src/map/keyframe.cpp
	src/map/frame.cpp
  src/map/landmark.cpp
  src/map/map_utils.cpp
  src/map/projection_2D.cpp
  src/map/camera.cpp
  src/opencv_utils.cpp
  src/read_from_launch.cpp
)
set(MAPPING_HEADER_FILES
	include/boris_drone/map/mapping_node.h
	include/boris_drone/map/map.h
	include/boris_drone/map/keyframe.h
	include/boris_drone/map/frame.h
  include/boris_drone/map/landmark.h
  include/boris_drone/map/map_utils.h
  include/boris_drone/map/projection_2D.h
  include/boris_drone/map/camera.h
  include/boris_drone/opencv_utils.h
  include/boris_drone/read_from_launch.h
)
set(COMPUTER_VISION_SOURCE_FILES
  src/computer_vision/processed_image.cpp
  src/computer_vision/target.cpp
)
set(COMPUTER_VISION_HEADER_FILES
  include/boris_drone/computer_vision/computer_vision.h
  include/boris_drone/computer_vision/processed_image.h
  include/boris_drone/computer_vision/target.h
)

## Declare a C++ executable
# add_executable(boris_drone_node src/boris_drone_node.cpp)
add_executable(controller_custom src/controller/controller_custom.cpp)
add_executable(bundle_adjuster src/map/bundle_adjuster.cpp src/opencv_utils.cpp)
add_executable(image_piper src/imagepiper.cpp)
add_executable(controller src/controller/controller.cpp)
add_executable(pose_estimation src/pose_estimation/pose_estimation.cpp)
add_executable(manual_pose_estimation src/pose_estimation/manual_pose_estimation.cpp)
add_executable(mapping_node ${MAPPING_SOURCE_FILES}         ${MAPPING_HEADER_FILES}
                            ${COMPUTER_VISION_SOURCE_FILES} ${COMPUTER_VISION_HEADER_FILES})
add_executable(computer_vision  src/computer_vision/image_processor.cpp include/boris_drone/computer_vision/image_processor.h
 ${COMPUTER_VISION_SOURCE_FILES} ${COMPUTER_VISION_HEADER_FILES}
 src/map/projection_2D.cpp src/opencv_utils.cpp src/read_from_launch.cpp)
add_executable(vision_gui  src/vision_gui/vision_gui.cpp  src/vision_gui/vision_gui.cpp)
add_executable(path_planning src/path_planning/path_planning.cpp)
add_executable(strategy src/strategy/strategy.cpp)
add_executable(multi_strategy src/multi_strategy/multi_strategy.cpp src/multi_strategy/drone_role.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(controller_custom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(bundle_adjuster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(image_piper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(imgproc2D ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pose_estimation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(manual_pose_estimation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(simple_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(mapping_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(computer_vision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(vision_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(path_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(strategy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(multi_strategy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(boris_drone_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(controller ${catkin_LIBRARIES})
target_link_libraries(bundle_adjuster ${catkin_LIBRARIES} ${CERES_LIBRARIES})
target_link_libraries(image_piper ${catkin_LIBRARIES})
target_link_libraries(controller_custom ${catkin_LIBRARIES})
target_link_libraries(pose_estimation ${catkin_LIBRARIES})
target_link_libraries(manual_pose_estimation ${catkin_LIBRARIES})
target_link_libraries(mapping_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} opencv_nonfree libvtkCommon.so libvtkFiltering.so)
target_link_libraries(computer_vision ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} opencv_nonfree)
target_link_libraries(vision_gui ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} opencv_nonfree)
target_link_libraries(path_planning ${catkin_LIBRARIES})
target_link_libraries(strategy ${catkin_LIBRARIES})
target_link_libraries(multi_strategy ${catkin_LIBRARIES})

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
# install(TARGETS boris_drone boris_drone_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_boris_drone.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
