project(mj2)
cmake_minimum_required(VERSION 2.8.3)

######################################################################
# EXTERNAL LIBRARIES
######################################################################

include(ExternalProject)

# Path for external libraries
IF(NOT (IS_DIRECTORY "${mj2_SOURCE_DIR}/external"))
  file(MAKE_DIRECTORY "${mj2_SOURCE_DIR}/external")
endif()
set(EXTERNAL_INSTALL_LOCATION "${mj2_SOURCE_DIR}/external")

# Eigen library
ExternalProject_Add(Eigen
  GIT_REPOSITORY https://github.com/RLovelett/eigen.git
  PREFIX ${EXTERNAL_INSTALL_LOCATION}/Eigen
  BUILD_COMMAND ""
  CONFIGURE_COMMAND ""
  INSTALL_COMMAND ""
)

include_directories(${EXTERNAL_INSTALL_LOCATION}/Eigen/src/Eigen)

# CImg library
ExternalProject_Add(CImg
  GIT_REPOSITORY https://github.com/dtschump/CImg.git
  PREFIX ${EXTERNAL_INSTALL_LOCATION}/CImg
  BUILD_COMMAND ""
  CONFIGURE_COMMAND ""
  INSTALL_COMMAND ""
)

include_directories(${EXTERNAL_INSTALL_LOCATION}/CImg/src/CImg)

# Catch library
ExternalProject_Add(Catch
  GIT_REPOSITORY https://github.com/philsquared/Catch.git
  PREFIX ${EXTERNAL_INSTALL_LOCATION}/Catch
  BUILD_COMMAND ""
  CONFIGURE_COMMAND ""
  INSTALL_COMMAND ""
)

include_directories(${EXTERNAL_INSTALL_LOCATION}/Catch/src/Catch/single_include)

# pybind11 library
ExternalProject_Add(pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  PREFIX ${EXTERNAL_INSTALL_LOCATION}/pybind11
  BUILD_COMMAND ""
  CONFIGURE_COMMAND ""
  INSTALL_COMMAND ""
)

include_directories(${EXTERNAL_INSTALL_LOCATION}/pybind11/src/pybind11/include)

##################################################################################
# PATHS
##################################################################################

# Path for output executable files
IF(NOT (IS_DIRECTORY "${mj2_SOURCE_DIR}/bin"))
  file(MAKE_DIRECTORY "${mj2_SOURCE_DIR}/bin")
endif()
set(EXECUTABLE_OUTPUT_PATH "${mj2_SOURCE_DIR}/bin")

# Base include directory
include_directories(${mj2_SOURCE_DIR})
#include_directories(${mj2_SOURCE_DIR}/image)
  
###################################################################################
# COMPILER FLAGS
###################################################################################

# Set a default build type for single-configuration
# CMake generators if no build type is set.
IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

# Select flags.
if ( ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang") OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU") OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel") )
    set(CMAKE_CXX_FLAGS         "-std=c++17 -Wall")
    set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -mtune=native")
    if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
       set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -Dsrandom=srand -Drandom=rand -D_USE_MATH_DEFINES")
    endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    message("Using Visual Studio, are you sure?")
endif()

message(STATUS "Compiler  = ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "System    = ${CMAKE_SYSTEM_NAME}")
message(STATUS "Prefix    = ${CMAKE_PREFIX_PATH}")
message(STATUS "Flags     = ${CMAKE_CXX_FLAGS}")
message(STATUS "Build     = ${CMAKE_BUILD_TYPE}")
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
   message(STATUS "R.Flags   = ${CMAKE_CXX_FLAGS_RELEASE}")
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
   message(STATUS "D.Flags   = ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
   message(STATUS "D.Flags   = ${CMAKE_CXX_FLAGS_DEBUG}")
endif()

##########################################################################################
# USEFUL FUNCTIONS
##########################################################################################
function(add_all_subdirectories)
	file(GLOB children RELATIVE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/${child})
			add_subdirectory(${child})
		endif()
	endforeach()
endfunction(add_all_subdirectories)

function(add_executable_and_test executable srcs)
	add_executable(${executable} ${srcs})
	add_test(NAME "test-${executable}" COMMAND ${executable})
endfunction(add_executable_and_test)

###########################################################################################
# TARGETS
###########################################################################################
enable_testing()
add_subdirectory(main)
