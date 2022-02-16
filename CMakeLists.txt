cmake_minimum_required(VERSION 3.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


### Configuration
set(MANHATTAN_ROOT        			 ${CMAKE_CURRENT_LIST_DIR})
set(MANHATTAN_EXTERNAL       		 ${MANHATTAN_ROOT}/3rd_party)
set(MANHATTAN_EXTERNAL_cminpack_DIR  ${MANHATTAN_EXTERNAL}/cminpack-1.3.6)
set(MANHATTAN_EXTERNAL_liblbfgs_DIR  ${MANHATTAN_EXTERNAL}/liblbfgs)
set(MANHATTAN_optimizer_DIR          ${MANHATTAN_ROOT}/optimizer)

add_subdirectory(3rd_party)
add_subdirectory(optimizer)

# request Easy3D
find_package(Easy3D REQUIRED)

add_executable(${PROJECT_NAME}
        main.cpp
        mesh_manhattan.h
        mesh_manhattan.cpp
        )

# The data directory
target_compile_definitions(${PROJECT_NAME} PRIVATE "DATA_DIR=\"${MANHATTAN_ROOT}/data\"")

target_include_directories(${PROJECT_NAME} PRIVATE ${MANHATTAN_optimizer_DIR})

# link to the viewer module of Easy3D
target_link_libraries(${PROJECT_NAME} easy3d::viewer optimizer)
