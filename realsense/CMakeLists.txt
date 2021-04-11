
# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(LOCATE_DEVICE)

find_package(OpenCV REQUIRED)
include_directories($(OpenCV_INCLUDE_DIRS))


find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()
# getting-started
include_directories(includes)
set(SOURCES src/locate_device.cpp)
#set(SOURCES src/get_pointcloud.cpp)
add_executable(locate_device ${SOURCES})
#add_executable(registration src/register.cpp)


target_link_libraries(locate_device ${REALSENSE2_FOUND})


find_package(OpenGL REQUIRED)
target_link_libraries(locate_device ${OPENGL_LIBRARIES})
find_package(glfw3 REQUIRED)
target_link_libraries(locate_device glfw)

target_link_libraries(locate_device ${DEPENDENCIES})
target_link_libraries(locate_device ${OpenCV_LIBS})
include_directories(locate_device ${ADDITIONAL_INCLUDE_DIRS})
target_link_libraries(locate_device -lpthread -lm)
#target_link_libraries(locate_device ${CMAKE_THREAD_LIBS_INIT})
set_target_properties (locate_device PROPERTIES FOLDER realsense)

install(
    TARGETS

    locate_device

    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)


