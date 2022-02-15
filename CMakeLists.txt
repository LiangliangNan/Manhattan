cmake_minimum_required(VERSION 3.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


### Configuration
set(SOLVER_SUITE_ROOT        			${CMAKE_CURRENT_LIST_DIR})
set(SOLVER_SUITE_EXTERNAL       		${SOLVER_SUITE_ROOT}/3rd_party)
set(SOLVER_SUITE_EXTERNAL_cminpack_DIR          ${SOLVER_SUITE_EXTERNAL}/cminpack-1.3.6)
set(SOLVER_SUITE_EXTERNAL_liblbfgs_DIR          ${SOLVER_SUITE_EXTERNAL}/liblbfgs)

set(SOLVER_SUITE_optimizer_DIR                  ${SOLVER_SUITE_ROOT}/optimizer)

add_subdirectory(3rd_party)
add_subdirectory(optimizer)

# request Easy3D
find_package(Easy3D REQUIRED)

add_executable(${PROJECT_NAME}
        main.cpp
        mesh_manhattan.h
        mesh_manhattan.cpp
        )

target_include_directories(${PROJECT_NAME} PRIVATE
        ${SOLVER_SUITE_optimizer_DIR}
        )

# link to the viewer module of Easy3D
target_link_libraries(${PROJECT_NAME} easy3d::viewer easy3d::algo_ext optimizer)
