find_package(assimp REQUIRED)
if (assimp_FOUND)
    link_directories(${ASSIMP_LIBRARY_DIRS})
    include_directories(${ASSIMP_INCLUDE_DIRS})
endif(assimp_FOUND)

add_executable(test-assimp assimp.cc)

if (assimp_FOUND)
     target_link_libraries(test-assimp ${ASSIMP_LIBRARIES})
endif(assimp_FOUND)
