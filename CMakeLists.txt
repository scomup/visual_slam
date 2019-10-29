cmake_minimum_required(VERSION 2.8.12)  # Ships with Ubuntu 14.04 (Trusty)

project(visual_slam)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O0 -g")
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3")

find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Ceres REQUIRED COMPONENTS SparseLinearAlgebraLibrary)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Torch REQUIRED)
find_package(yaml-cpp REQUIRED)


find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs  message_filters cv_bridge  rosbag)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_BINARY_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})

file(GLOB_RECURSE ALL_SRCS "src/*.cc")
message(${ALL_SRCS})

add_executable(${PROJECT_NAME}  ${ALL_SRCS})


target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
${Boost_LIBRARIES}
${PCL_LIBRARIES}
${CERES_LIBRARIES}
${OpenCV_LIBS}
${YAML_CPP_LIBRARIES}
${TORCH_LIBRARIES}
pthread
dl
)

