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
add_subdirectory(Manhattan)