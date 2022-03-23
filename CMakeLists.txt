cmake_minimum_required(VERSION 2.8.3)
# to make CMake shut up about some fckin policies
cmake_policy(SET CMP0048 NEW)
project(uav_detect VERSION 666)

# Override CXX flags inherited from workspace, if precompiled PCL binaries from debian repos are used
if (DEFINED ENV{PCL_CROSS_COMPILATION})
  set(PCL_CROSS_COMPILATION $ENV{PCL_CROSS_COMPILATION})
else()
  set(PCL_CROSS_COMPILATION "false")
endif()
if(${PCL_CROSS_COMPILATION} STREQUAL "false")
  message("Using precompiled PCL binaries from debian repos. Overriding catkin workspace CMAKE_CXX_FLAGS.")
  set(CMAKE_CXX_FLAGS "-std=c++17")
else()
  message("Using custom-built PCL binaries. Inheriting all CMAKE_CXX_FLAGS from catkin workspace.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  mrs_msgs
  sensor_msgs
  dynamic_reconfigure
  cv_bridge
  image_transport
  image_geometry
  message_generation
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  tf2_eigen
  tf2_geometry_msgs
  visualization_msgs
  ouster_ros
  mrs_lib
  mrs_pcl_tools
  nodelet
  pcl_ros
  pcl_conversions
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options("-Wall")
add_compile_options("-Wextra")
add_compile_options("-DNDEBUG")
# add_compile_options("-DEIGEN_INITIALIZE_MATRICES_BY_NAN")
# add_compile_options("-DDISABLE_PARALLELIZATION")
add_compile_options("-g")
# add_definitions(-fsanitize=address,undefined,alignment,bounds)
# add_link_options(-fsanitize=address,undefined,alignment,bounds)
# add_definitions(-fsanitize=thread)
# add_link_options(-fsanitize=thread)

FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF()

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

add_message_files(DIRECTORY msgs FILES
  Detection.msg
  DetectionStamped.msg
  Detections.msg
  Status.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  config/DetectionParams.cfg
  config/LocalizationParams.cfg
  )

catkin_package(
  CATKIN_DEPENDS message_runtime pcl_ros image_transport mrs_lib # roscpp rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_library(PCLDetector src/pcl_detect_nodelet.cpp src/voxel_map.cpp src/voxel_grid_weighted.cpp src/pcl_loader.cpp)
add_dependencies(PCLDetector
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(PCLDetector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_include_directories(PCLDetector SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})

add_library(PCLSelfLocalizator src/pcl_selfloc_nodelet.cpp src/pcl_loader.cpp)
add_dependencies(PCLSelfLocalizator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(PCLSelfLocalizator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_include_directories(PCLSelfLocalizator SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})

add_library(MaskCreator src/mask_creator.cpp)
add_dependencies(MaskCreator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(MaskCreator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_include_directories(MaskCreator SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
