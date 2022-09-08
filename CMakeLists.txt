cmake_minimum_required(VERSION 3.14)
project(opti LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV 4.5.5 REQUIRED)
find_package(Ceres REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/ceres/install/lib/cmake/Ceres)
find_package(Eigen3 3.4.0 REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
        # module/include
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        )

target_link_libraries(${PROJECT_NAME} PUBLIC
        # module
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        Eigen3::Eigen
        )
