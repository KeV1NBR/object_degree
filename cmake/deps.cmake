# Project Dependences Configuration

# Backup and set build type to release
if(NOT MSVC)
    set(CMAKE_BUILD_TYPE_BAK ${CMAKE_BUILD_TYPE})
    set(CMAKE_BUILD_TYPE Release)
endif()

# Include subdirectories
include_directories(${DEPS_PATHS})

# Find other dependences
set(OpenCV_STATIC OFF CACHE BOOL "Using OpenCV static linking library")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Darknet)
include_directories(${DARKNET_INCLUDE_DIRS})

find_package(realsense2 REQUIRED)
include_directories(${REALSENSE_INCLUDE_DIR})

find_package(OpenMP)

# Add subdirectory
foreach(DEPS_PATH ${DEPS_PATHS})
    add_subdirectory(${DEPS_PATH})
endforeach()

# Restore origin build type
if(NOT MSVC)
    set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE_BAK})
endif()

