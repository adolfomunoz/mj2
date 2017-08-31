add_executable(depthmap depthmap.cc)
target_compile_definitions(depthmap PRIVATE ${cimg_defs})
target_link_libraries(depthmap ${cimg_libs})
