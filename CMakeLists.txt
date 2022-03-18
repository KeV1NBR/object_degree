cmake_minimum_required(VERSION 2.8)

# Disable in-source builds to prevent source tree corruption.
if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
    message(FATAL_ERROR "\nFATAL: In-source builds are not allowed.\n"
        "You should create a separate directory for build files."
        )
endif()

# Set variables
set(PROJECT_NAME obj_deg)
set(PROJECT_LANGUAGE C CXX)

# Environment variables
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation directory")
set(CMAKE_CXX_FLAGS "-std=c++11 -rdynamic -g")
option(BUILD_UTIL "Enable building test" ON)
option(BUILD_SHARED_LIBS "Build shared library" ON)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Build configure" FORCE)
if(DEFINED CMAKE_BUILD_TYPE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
endif()
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Let ycm can catch compile commands.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Dependences paths
set(DEPS_PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/realsense_lib/lib
    )

# Library paths
set(LIB_PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/
    )

if(${BUILD_UTIL})
    set(UTIL_PATHS ${UTIL_PATHS} ${CMAKE_CURRENT_SOURCE_DIR}/test)
endif()

# Set project
project(${PROJECT_NAME} ${PROJECT_LANGUAGE})

# Include cmake files
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/deps.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/lib.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/util.cmake)

