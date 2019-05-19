cmake_minimum_required(VERSION 2.8)
project (tracker)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/src/)
SET(HDR_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/include/)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/)

FILE(GLOB_RECURSE TRACKER_SRC "${SRC_FOLDER}/*.c*")
FILE(GLOB_RECURSE TRACKER_HDR "${HDR_FOLDER}/*.h*")

SET(TRACKER_SRC_FILES	${CMAKE_CURRENT_SOURCE_DIR}/src/stream/RGBDStream.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/stream/DetectionStream.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/Detector.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
			${TRACKER_HDR})

find_package(OpenCV REQUIRED)

#include_directories(${OpenCV_INCLUDE_DIRS})

#link_directories(${OpenCV_LIBRARY_DIRS})

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIRS})

#include_directories(${HDR_FOLDER})

add_definitions(-std=c++17 -DCORE_COMPILATION -Wno-ignored-attributes)

add_executable(main ${TRACKER_SRC_FILES})

target_link_libraries(main stdc++fs ${OpenCV_LIBS})
