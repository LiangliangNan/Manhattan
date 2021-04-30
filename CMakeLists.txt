cmake_minimum_required(VERSION 3.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(Test)

# request Easy3D
find_package(Easy3D REQUIRED)

add_executable(Test main.cpp)

# link to the viewer module of Easy3D
target_link_libraries(Test easy3d::viewer easy3d::algo_ext)

#
#
#set_target_properties(${PROJECT_NAME} PROPERTIES
#        MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME}
#        MACOSX_BUNDLE TRUE
#        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/app.plist"
#        )
