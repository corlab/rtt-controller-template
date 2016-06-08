#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.8.3
#
cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_PREFIX_PATH /vol/cogimon)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(RTTControllerTemplate)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

#find_package(Eigen3)
#add_definitions(${EIGEN3_DEFINITIONS})
#include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
#find_package(RST-RT REQUIRED)
#message(STATUS "RST-RT version: ${RST-RT_VERSION}")
#include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
#add_definitions(${RST-RT_CFLAGS})
#link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})

find_package(rtt-core-extensions REQUIRED)
include_directories(${RTT-CORE-EXTENSIONS_INCLUDE_DIRS})
link_directories(${RTT-CORE-EXTENSIONS_LIBRARY_DIRS})

#find_package(RTTToolchainTools)
#message(STATUS "RTTToolchainTools version: ${RTTToolchainTools_VERSION}")	
#include_directories(BEFORE SYSTEM ${RTTToolchainTools_INCLUDE_DIRS})	
#add_definitions(${RTTToolchainTools_CFLAGS})
#link_directories(${RTTToolchainTools_LIBRARY_DIR})

find_package(orocos_kdl REQUIRED)
message(STATUS "orocos_kdl version: ${orocos_kdl_VERSION}")
include_directories(BEFORE SYSTEM ${orocos_kdl_INCLUDE_DIRS})	
add_definitions(${orocos_kdl_CFLAGS})
link_directories(${orocos_kdl_LIBRARY_DIR})

find_package(PkgConfig REQUIRED)
pkg_search_module(TINYXML REQUIRED tinyxml)
if(NOT TINYXML_INCLUDE_DIR)
	find_path("/usr/include" tinyxml.h)
	find_library(TINYXML_LIBRARY NAMES tinyxml)
	set(TINYXML_LIBRARIES ${TINYXML_LIBRARY})
    set(TINYXML_INCLUDE_DIRS ${TINYXML_INCLUDE_DIR})
    set(TINYXML_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
endif()
message(STATUS "TINYXML_VERSION: ${TINYXML_VERSION}")
include_directories(BEFORE SYSTEM ${TINYXML_INCLUDE_DIRS})
link_directories(${TINYXML_LIBRARY_DIR})
message(STATUS "TINYXML_LIBRARIES: ${TINYXML_LIBRARIES}")
message(STATUS "TINYXML_LIBRARY_DIR: ${TINYXML_LIBRARY_DIR}")
message(STATUS "TINYXML_INCLUDE_DIR: ${TINYXML_INCLUDE_DIRS}")
message(STATUS "TINYXML_LIBRARY: ${TINYXML_LIBRARY}")

# Rest of the configuration is in src/
add_subdirectory( src )

#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
