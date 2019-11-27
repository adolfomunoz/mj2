add_executable(colormap colormap.cc)
target_compile_definitions(colormap PRIVATE ${cimg_defs})
target_link_libraries(colormap ${cimg_libs})
