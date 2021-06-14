cmake_minimum_required(VERSION 3.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

# request Easy3D
find_package(Easy3D REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

# link to the viewer module of Easy3D
target_link_libraries(${PROJECT_NAME} easy3d::viewer easy3d::algo_ext)

#
#
#set_target_properties(${PROJECT_NAME} PROPERTIES
#        MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME}
#        MACOSX_BUNDLE TRUE
#        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/app.plist"
#        )
