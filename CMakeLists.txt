cmake_minimum_required(VERSION 2.8)
project (tracker)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenGL_GL_PREFERENCE "LEGACY")

SET(SRC_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/src/)
SET(HDR_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/include/)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/)

FILE(GLOB_RECURSE TRACKER_SRC "${SRC_FOLDER}/*.c*")
FILE(GLOB_RECURSE TRACKER_HDR "${HDR_FOLDER}/*.h*")

SET(TRACKER_SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/stream/RGBDStream.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/stream/DetectionStream.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/viewer/GLViewer.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/Detector.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)


# Add thread mutex
SET(SPECIAL_OS_LIBS "pthread" "X11")
add_definitions(-std=c++0x -g -O3 -pthread -Wno-ignored-attributes -Wno-write-strings)

# Required Packages
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenCV REQUIRED)

find_package(CUDA REQUIRED)
find_package(OpenGL)
find_package(Eigen3 REQUIRED)

# Package includes
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

link_directories(${GLEW_LIBRARY_DIRS})
#link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARIES})
link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${Eigen_LIBRARY_DIRS})

add_executable(tracker ${TRACKER_HDR} ${TRACKER_SRC_FILES})

#set_target_properties(tracker PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(tracker stdc++fs
                    ${GLUT_LIBRARY}
                    ${GLUT_glut_LIBRARY}
				    ${GLEW_LIBRARY}
                    ${SPECIAL_OS_LIBS}
				    ${OpenCV_LIBRARIES}
				    ${OpenGL_LIBRARIES}
                    ${OPENGL_gl_LIBRARY}
				    ${Eigen_LIBRARIES}
				    ${CUDA_CUDA_LIBRARY})
