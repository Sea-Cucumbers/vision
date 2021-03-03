
# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(REGISTRATION)

find_package(OpenCV REQUIRED)
include_directories($(OpenCV_INCLUDE_DIRS))


find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()
# getting-started
include_directories(includes)
set(SOURCES src/boson_image.cpp src/register.cpp)
#set(SOURCES src/get_pointcloud.cpp)
add_executable(registration ${SOURCES})
#add_executable(registration src/register.cpp)


target_link_libraries(registration ${REALSENSE2_FOUND})


find_package(OpenGL REQUIRED)
target_link_libraries(registration ${OPENGL_LIBRARIES})
find_package(glfw3 REQUIRED)
target_link_libraries(registration glfw)

target_link_libraries(registration ${DEPENDENCIES})
target_link_libraries(registration ${OpenCV_LIBS} "-lpopt")
include_directories(registration ${ADDITIONAL_INCLUDE_DIRS})
target_link_libraries(registration -lpthread -lm)
#target_link_libraries(registration ${CMAKE_THREAD_LIBS_INIT})
set_target_properties (registration PROPERTIES FOLDER ThermalRegistration)

install(
    TARGETS

    registration

    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)


