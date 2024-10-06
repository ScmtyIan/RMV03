cmake_minimum_required(VERSION 3.10.0)
project(task01)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(include ${OpenCV_INCLUDE_DIRS})

aux_source_directory(src SRC)
add_library(SRCS_STATIC STATIC ${SRC})

add_executable(task main.cpp ./src/DataFitting.cpp)
target_link_libraries(task ${OpenCV_LIBS})
target_link_libraries(task Ceres::ceres)
target_link_libraries(task SRCS_STATIC)


# 静态库不过分把



