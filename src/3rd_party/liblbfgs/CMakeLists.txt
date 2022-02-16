cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(3rd_liblbfgs)


set(liblbfgs_HEADERS
    include/lbfgs.h
    )

set(liblbfgs_SOURCES
    lib/arithmetic_ansi.h
    lib/arithmetic_sse_double.h
    lib/arithmetic_sse_float.h
    lib/lbfgs.c
    )


add_library(3rd_liblbfgs STATIC ${liblbfgs_SOURCES} ${liblbfgs_HEADERS})

set_target_properties(3rd_liblbfgs PROPERTIES FOLDER "3rd_party")

target_include_directories(3rd_liblbfgs PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    )


target_compile_definitions(3rd_liblbfgs PRIVATE USE_SSE __SSE__ __SSE2__ __SSE3__ HAVE_EMMINTRIN_H HAVE_XMMINTRIN_H)


