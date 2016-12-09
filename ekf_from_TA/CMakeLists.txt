cmake_minimum_required(VERSION 2.8.3)
project(ekf)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3  REQUIRED)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    )

link_directories(${PCL_LIBRARY_DIRS})
catkin_package(
    )

#add_library(ekf src/ekf.cpp)

add_executable(ekf_node
    src/ekf_node.cpp src/ekf.cpp src/pose.cpp )
#
target_link_libraries(ekf_node 
		    ${catkin_LIBRARIES}
		    ${PCL_LIBRARIES}
)    
