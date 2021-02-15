cmake_minimum_required(VERSION 3.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

find_package(Easy3D REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} easy3d::algo_ext easy3d::viewer)
