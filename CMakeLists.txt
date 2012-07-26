##########################################################################################
# Part of PhD thesis of Anne van Rossum, concerning action-perception loops. This makefile
# is part of a cmake build system. 
# The code falls under the Lesser General Public License (LGPL GNU v3).

# Author:	 Anne C. van Rossum (Almende B.V.)
# Date: 	 Apr. 16, 2012
#
# Copyright © 2012 Anne van Rossum <anne@almende.com>
##########################################################################################

IF(WIN32)
cmake_minimum_required(VERSION 2.6)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

##########################################################################################
# The name of the final binary
SET(PROJECT_NAME "ParticleFilter")

# The name of the path from the parent directory
SET(TESTBENCH_PATH "test")

##########################################################################################

# Set the name
PROJECT(${PROJECT_NAME})

# Find packages
#FIND_PACKAGE(YARP REQUIRED)

# Header files
#INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

# Shared libraries
#SET(LIBS ${LIBS} ${YARP_LIBRARIES})

# Some debug information
MESSAGE("${PROJECT_NAME} is using CXX flags: ${CMAKE_CXX_FLAGS}")
MESSAGE ("Libraries included: \"${LIBS}\"")

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c ${TESTBENCH_PATH}/*.c ${TESTBENCH_PATH}/*.cpp)
FILE(GLOB folder_header inc/*.h inc/*.hpp ${TESTBENCH_PATH}/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Automatically add include directories if needed.
FOREACH(header_file ${folder_header})
  GET_FILENAME_COMPONENT(p ${header_file} PATH)
  INCLUDE_DIRECTORIES(${p})
ENDFOREACH(header_file ${folder_header})

# Testing
# enable_testing()
#add_subdirectory(test)

# Set up our main executable.
IF (folder_source)
   ADD_EXECUTABLE(${PROJECT_NAME} ${folder_source} ${folder_header})
   TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LIBS})
   install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)   
ELSE (folder_source)
  MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

