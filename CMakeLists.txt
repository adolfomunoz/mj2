project(mj2)
cmake_minimum_required(VERSION 3.0)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include(Paths)
include(External)
include(Compiler)

#########################################################################################
# USEFUL FUNCTIONS
#########################################################################################
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

#########################################################################################
# TARGETS
#########################################################################################
enable_testing()
add_subdirectory(main)
