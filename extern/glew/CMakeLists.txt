project(glew C)

add_library(glew STATIC src/glew.c)

target_compile_definitions(glew PUBLIC GLEW_STATIC)

target_include_directories(glew PUBLIC include ${OPENGL_INCLUDE_DIR})

target_link_libraries(glew PUBLIC ${OPENGL_LIBRARIES})
