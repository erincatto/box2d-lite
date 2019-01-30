cmake_minimum_required(VERSION 3.8)

project(box2d-lite LANGUAGES CXX)

add_subdirectory(src)

option(BOX2D_BUILD_SAMPLES "Build the box2d-lite sample program" ON)

if (BOX2D_BUILD_SAMPLES)

	find_package(OpenGL REQUIRED)

	add_subdirectory(extern/glfw)
	add_subdirectory(extern/imgui)
	add_subdirectory(samples)

	# default startup project for Visual Studio
	if (MSVC)
		set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT samples)
	endif()

endif()
