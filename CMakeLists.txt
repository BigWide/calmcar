cmake_minimum_required(VERSION 2.8.3)
project(calmcar)


EXECUTE_PROCESS(COMMAND uname -s
        OUTPUT_VARIABLE SYSNAME)
STRING(STRIP ${SYSNAME} SYSNAME)

EXECUTE_PROCESS(COMMAND uname -m
        OUTPUT_VARIABLE MACHINE)
STRING(STRIP ${MACHINE} MACHINE)

SET(PLATFORM ${SYSNAME}-${MACHINE})

MESSAGE("Platform: ${PLATFORM}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

ADD_COMPILE_OPTIONS(-std=c++11 -O3 -Wall -Wunused-variable)

#find_package(OpenCV)
find_package(OpenCV 3 REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  sensor_msgs
  cv_bridge
  image_transport
)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  sensor_msgs
#)
###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
)

## Build ##
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/tcp)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/interface)
INCLUDE_DIRECTORIES(/usr/local/include)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/data)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/protobuf-3.6.1/include)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib3/${PLATFORM})
set(PROTOBUF_LIB ${PROJECT_SOURCE_DIR}/protobuf-3.6.1/lib/${PLATFORM})
link_directories(${PROTOBUF_LIB})
SET(SRC_DIR
  ${PROJECT_SOURCE_DIR}/src/tcp/tcp_client.cpp
  ${PROJECT_SOURCE_DIR}/src/tcp/tcp_connection.cpp
  ${PROJECT_SOURCE_DIR}/src/data/calmcar.pb.cc
  ${PROJECT_SOURCE_DIR}/src/interface/calmcar_sdk.cpp
  ${PROJECT_SOURCE_DIR}/src/calmcar_pub.cpp
  )
add_executable(calmcar_pub ${SDK_TEST_SRC} ${SRC_DIR})
IF (${PLATFORM} STREQUAL "Linux-aarch64")

target_link_libraries(calmcar_pub jpeg ${OpenCV_LIBS} pthread ${PROTOBUF_LIB}/libprotobuf.a ${catkin_LIBRARIES})

ELSE()

target_link_libraries(calmcar_pub jpeg ${OpenCV_LIBS} pthread ${PROTOBUF_LIB}/libprotobuf.a ${catkin_LIBRARIES})

ENDIF()

SET(SRC_DIR_ROI
  ${PROJECT_SOURCE_DIR}/src/tcp/tcp_client.cpp
  ${PROJECT_SOURCE_DIR}/src/tcp/tcp_connection.cpp
  ${PROJECT_SOURCE_DIR}/src/data/calmcar.pb.cc
  ${PROJECT_SOURCE_DIR}/src/interface/calmcar_sdk.cpp
  ${PROJECT_SOURCE_DIR}/src/calmcar_pub_roi.cpp
  )
add_executable(calmcar_pub_roi ${SDK_TEST_SRC} ${SRC_DIR_ROI})
IF (${PLATFORM} STREQUAL "Linux-aarch64")

target_link_libraries(calmcar_pub_roi jpeg ${OpenCV_LIBS} pthread ${PROTOBUF_LIB}/libprotobuf.a ${catkin_LIBRARIES})

ELSE()

target_link_libraries(calmcar_pub_roi jpeg ${OpenCV_LIBS} pthread ${PROTOBUF_LIB}/libprotobuf.a ${catkin_LIBRARIES})

ENDIF()
