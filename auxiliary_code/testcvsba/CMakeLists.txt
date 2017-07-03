cmake_minimum_required(VERSION 2.8.3)
project(testcvsba)

LIST(APPEND CMAKE_MODULE_PATH "~/cvsba-1.0.0/build")


find_package( OpenCV REQUIRED core nonfree )# ocl gpu)

find_package( cvsba REQUIRED )

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
	)

add_executable(testcvsba testcvsba.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(testcvsba ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(boris_drone_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(testcvsba ${OpenCV_LIBRARIES} opencv_nonfree libvtkCommon.so libvtkFiltering.so ${cvsba_LIBS})
