add_executable(normalmap normalmap.cc)
target_compile_definitions(normalmap PRIVATE ${cimg_defs})
target_link_libraries(normalmap ${cimg_libs})
