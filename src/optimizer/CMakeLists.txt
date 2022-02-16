cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


set(optimizer_HEADERS
        optimizer_common.h
        optimizer_lbfgs.h
        optimizer_lm.h
        )

set(optimizer_SOURCES
        optimizer_lbfgs.cpp
        optimizer_lm.cpp
        )


add_library(${PROJECT_NAME} STATIC ${optimizer_SOURCES} ${optimizer_HEADERS})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "solvers")

target_compile_definitions(${PROJECT_NAME} PRIVATE OPTIMIZER_EXPORTS)

target_include_directories(${PROJECT_NAME} PRIVATE
        ${MANHATTAN_optimizer_DIR}
        ${MANHATTAN_EXTERNAL_cminpack_DIR}
        ${MANHATTAN_EXTERNAL_liblbfgs_DIR}
        )

target_link_libraries(${PROJECT_NAME} PRIVATE 3rd_liblbfgs 3rd_cminpack)
