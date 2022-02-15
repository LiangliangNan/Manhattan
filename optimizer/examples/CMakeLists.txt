cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(optimizer_example)


add_executable(optimizer_example
    main.cpp
    examples_optimizer_lbfgs.cpp
    examples_optimizer_lm.cpp
        )

set_target_properties(optimizer_example PROPERTIES FOLDER "examples")


target_include_directories(${PROJECT_NAME} PRIVATE
    ${SOLVER_SUITE_optimizer_DIR}
)


target_link_libraries(optimizer_example optimizer)
