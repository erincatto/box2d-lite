project(samples LANGUAGES CXX)

set (SAMPLE_SOURCE_FILES
	imgui_impl_glfw.cpp
        imgui_impl_opengl3.cpp
	main.cpp)

set (SAMPLE_HEADER_FILES
	imgui_impl_glfw.h
        imgui_impl_opengl3.h)

add_executable(samples ${SAMPLE_SOURCE_FILES} ${SAMPLE_HEADER_FILES})

target_compile_definitions(samples PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLEW)
target_link_libraries(samples PUBLIC box2d-lite imgui)

if(EMSCRIPTEN)
    set_target_properties(samples PROPERTIES
        SUFFIX .html)
    set(SAMPLE_OPTIONS "-s WASM=1 -s FULL_ES2=1 -s USE_GLFW=3")
    set_target_properties(samples PROPERTIES
        COMPILE_FLAGS "${SAMPLE_OPTIONS}")
    set_target_properties(samples PROPERTIES
        LINK_FLAGS "${SAMPLE_OPTIONS}")
else()
    target_include_directories(samples PUBLIC ${OPENGL_INCLUDE_DIR})
    target_link_libraries(samples PUBLIC glew glfw ${OPENGL_LIBRARIES})
endif()
