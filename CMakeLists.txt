cmake_minimum_required(VERSION 3.1)


# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)


get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


find_package(Easy3D COMPONENTS core REQUIRED)
message(STATUS "EASY3D_INCLUDE_DIRS: ${EASY3D_INCLUDE_DIRS}")
message(STATUS "EASY3D_LIBRARIES: ${EASY3D_LIBRARIES}")

add_executable(${PROJECT_NAME} main.cpp)


if (APPLE)
    ## Note: On macOS, high-DPI support is enabled by settings in the Info.plist file. Make sure they are present.
    ## For more information, see: http://doc.qt.io/qt-5/highdpi.html
    ## Enable high DPI support by using my Info.plist file in which I added NSHighResolutionCapable to be true.
    set_target_properties(${PROJECT_NAME} PROPERTIES
            MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME}
            MACOSX_BUNDLE TRUE
            MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/app.plist"
            )
endif ()

target_include_directories(${PROJECT_NAME} PRIVATE ${EASY3D_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${EASY3D_LIBRARIES})
