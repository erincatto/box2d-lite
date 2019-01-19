cmake_minimum_required(VERSION 3.8)

project(box2d-lite LANGUAGES CXX)

add_subdirectory(src)

option(BOX2D_BUILD_SAMPLES "Build the box2d-lite sample program" ON)

if (BOX2D_BUILD_SAMPLES)

	find_package(OpenGL REQUIRED)

	add_subdirectory(extern/glfw)
	add_subdirectory(extern/imgui)

	# box2d-lite samples
	set (SAMPLE_SOURCE_FILES
		samples/imgui_impl_glfw.cpp
		samples/imgui_impl_opengl2.cpp
		samples/main.cpp)

	set (SAMPLE_HEADER_FILES
		samples/imgui_impl_glfw.h
		samples/imgui_impl_opengl2.h)

	add_executable(samples ${SAMPLE_SOURCE_FILES} ${SAMPLE_HEADER_FILES})
	target_include_directories(samples PUBLIC ${OPENGL_INCLUDE_DIR})
	target_link_libraries(samples PUBLIC box2d-lite glfw imgui ${OPENGL_LIBRARIES})

	# default startup project for Visual Studio
	if (MSVC)
		set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT samples)
	endif()

endif()
