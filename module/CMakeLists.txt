cmake_minimum_required(VERSION 3.14)
project(module LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MODULE_SOURCE_FILES
        src/compute.cpp
        src/optimization.cpp
        )

find_package(OpenCV 4.5.5 REQUIRED)
find_package(Ceres REQUIRED HINTS ${PROJECT_SOURCE_DIR}/Thirdparty/ceres/install/lib/cmake/Ceres)
find_package(Eigen3 3.4.0 REQUIRED)

add_library(${PROJECT_NAME} SHARED
        ${MODULE_SOURCE_FILES}
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        )

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        Eigen3::Eigen
        )

