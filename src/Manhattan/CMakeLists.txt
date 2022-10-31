cmake_minimum_required(VERSION 3.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

# request Easy3D
find_package(Easy3D REQUIRED)
if (Easy3D_FOUND)
    # if you need to the check the version: VERSION_LESS, VERSION_GREATER, VERSION_EQUAL, VERSION_LESS_EQUAL, or VERSION_GREATER_EQUAL
    if (Easy3D_VERSION VERSION_LESS "2.0")
        message(WARNING "The minimum version of Easy3D required is v2.0")
    endif ()
endif ()

add_executable(${PROJECT_NAME}
        main.cpp
        manhattan.h
        manhattan.cpp
        )

target_include_directories(${PROJECT_NAME} PRIVATE ${MANHATTAN_optimizer_DIR})

# link to the viewer module of Easy3D
target_link_libraries(${PROJECT_NAME} easy3d::viewer optimizer)
