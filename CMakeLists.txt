cmake_minimum_required(VERSION 2.8.3)
project(supertoroid)

## Compile as C++11, supported in ROS Kinetic and newer
## ALBA: change to c++14 for compatibility with PCR
##add_compile_options(-std=c++11)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  pcl_ros
  pcl_conversions
  eigen_conversions
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   st.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
      include
  LIBRARIES
      st_sampling
      st_fitting
      st_segmentation
      util
  CATKIN_DEPENDS roscpp message_runtime
  DEPENDS system_lib
)


IF (PCL_FOUND)
  INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
  LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
  ADD_DEFINITIONS(${PCL_DEFINITIONS})
ENDIF (PCL_FOUND)

include_directories(include ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIR} )
include_directories(${catkin_INCLUDE_DIRS} ${freenect2_INCLUDE_DIRS})

add_library(st_sampling  src/supertoroid/st_sampling.cpp)
add_library(st_fitting  src/supertoroid/st_fitting.cpp)
add_library(st_segmentation src/supertoroid/st_segmentation.cpp)
add_library(util  src/supertoroid/util.cpp)

add_dependencies(st_fitting supertoroid_generate_messages_cpp )
add_dependencies(st_sampling supertoroid_generate_messages_cpp )

target_link_libraries(st_sampling  ${catkin_LIBRARIES}  ${PCL_LIBRARY_DIRS} pcl_visualization)
target_link_libraries(st_fitting  ${catkin_LIBRARIES}  ${PCL_LIBRARY_DIRS} pcl_visualization)
target_link_libraries(st_segmentation  ${catkin_LIBRARIES}  ${PCL_LIBRARY_DIRS} pcl_visualization)
target_link_libraries(util  ${catkin_LIBRARIES}  ${PCL_LIBRARY_DIRS} pcl_visualization)


install(TARGETS st_sampling st_segmentation st_fitting util
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#executables for tests
add_executable(st_sampling_test_pcd src/test/st_sampling_test_pcd.cpp)
add_dependencies(st_sampling_test_pcd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(st_sampling_test_pcd st_sampling  ${catkin_LIBRARIES})


add_executable(st_sampling_test_big src/test/st_sampling_test_big.cpp)
add_dependencies(st_sampling_test_big ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(st_sampling_test_big st_sampling  ${catkin_LIBRARIES})

add_executable(st_segmentation_test_pcd src/test/st_segmentation_test_pcd.cpp)
add_dependencies(st_segmentation_test_pcd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(st_segmentation_test_pcd st_segmentation  ${catkin_LIBRARIES})

add_executable(st_fitting_test_pcd src/test/st_fitting_test_pcd.cpp)
add_dependencies(st_fitting_test_pcd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(st_fitting_test_pcd st_fitting st_sampling ${catkin_LIBRARIES} pcl_visualization)

add_executable(pcd_viewer src/test/pcd_viewer.cpp)
add_dependencies(pcd_viewer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd_viewer ${catkin_LIBRARIES} pcl_visualization)

add_executable(sampling_test_pcd_st src/test/sampling_test_pcd_st.cpp)
add_dependencies(sampling_test_pcd_st ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sampling_test_pcd_st st_sampling ${catkin_LIBRARIES})


#add_executable(openni_grabber src/test/openni_grabber.cpp)
#target_link_libraries(openni_grabber  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
