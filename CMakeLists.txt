cmake_minimum_required(VERSION 2.8.12)
project(PerceptionNode)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

set(IMAGE_PROC_MAJOR_VERSION 0)
set(IMAGE_PROC_MINOR_VERSION 1)
set(IMAGE_PROC_PATCH_VERSION 0)
set(IMAGE_PROC_VERSION ${IMAGE_PROC_MAJOR_VERSION}.${IMAGE_PROC_MINOR_VERSION}.${IMAGE_PROC_PATCH_VERSION}  )
set(IMAGE_PROC_SOVERSION ${IMAGE_PROC_MAJOR_VERSION}.${IMAGE_PROC_MINOR_VERSION}
)

#########################################################################
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
  message(STATUS "No build type selected, default to ${CMAKE_BUILD_TYPE}")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -fPIC")
else(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fPIC")
endif(CMAKE_BUILD_TYPE STREQUAL "Release")
#########################################################################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  image_transport
  std_msgs
  sensor_msgs
  geometry_msgs
  GoogleDeps
  Shm
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

list(APPEND CMAKE_PREFIX_PATH ${GoogleDeps_SOURCE_PREFIX}/share/cartographer)

find_package(cartographer REQUIRED)

include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")

set(BUILD_SHARED_LIBS OFF)
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
google_initialize_cartographer_project()
google_enable_testing()

include(FindPkgConfig)

find_package(LuaGoogle REQUIRED)
find_package(Ceres REQUIRED)

set(CUDA_root "/usr/local/cuda")
set(CUDA_INCLUDE_DIR ${CUDA_root}/include)
find_library(CUDART cudart "${CUDA_root}/lib64")
find_library(CUBLAS cublas "${CUDA_root}/lib64")
find_library(CURAND curand "${CUDA_root}/lib64")
find_library(CUDNN cudnn "${CUDA_root}/lib64")
set(CUDA_LIBS ${CUDART} ${CUBLAS} ${CURAND} ${CUDNN})

set(Darknet_LIB
  ${PROJECT_SOURCE_DIR}/lib/libdarknet.so
)

include(FindPkgConfig)

find_package(LuaGoogle REQUIRED)
find_package(Ceres REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
                 tf2_ros
                 std_msgs
                 sensor_msgs
                 geometry_msgs
                 image_transport
  DEPENDS Boost OpenCV
)

###########
## Build ##
###########

#if (CMAKE_BUILD_TYPE STREQUAL Debug)
  add_definitions(-DDEBUG)
#endif()

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}Main.cc
  src/${PROJECT_NAME}.cc
  src/ImageProc.cc
  src/DarknetDetector.cc
  src/Observation.cc
  src/tf_bridge.cc
  src/time_conversion.cc
)

target_link_libraries(${PROJECT_NAME}
  cartographer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Darknet_LIB}
  ${CUDA_LIBS}
)


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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ImageProc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
