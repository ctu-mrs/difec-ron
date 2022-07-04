cmake_minimum_required(VERSION 2.8.3)
# to make CMake shut up about some fckin policies
cmake_policy(SET CMP0048 NEW)
project(difec_ron VERSION 666)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  message_generation
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  tf2_eigen
  tf2_geometry_msgs
  visualization_msgs
  mrs_lib
  nodelet
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

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

generate_dynamic_reconfigure_options(
  config/FormationControlParams.cfg
)

add_message_files(DIRECTORY msg FILES
  FormationState.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime mrs_lib # roscpp rospy std_msgs
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
)

add_library(FormationControl src/formation_control.cpp)
add_dependencies(FormationControl
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(FormationControl
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(FormationErrorCalculator src/formation_error_calculator.cpp)
add_dependencies(FormationErrorCalculator ${catkin_LIBRARIES})
target_link_libraries(FormationErrorCalculator
  ${catkin_LIBRARIES}
  )
