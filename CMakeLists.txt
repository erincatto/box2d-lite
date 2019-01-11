cmake_minimum_required(VERSION 3.8)
project(box2d-lite LANGUAGES CXX)

set(SOURCE_FILES
	src/Arbiter.cpp
	src/Body.cpp
	src/Collide.cpp
	src/Joint.cpp
	src/World.cpp)

set(HEADER_FILES
	include/box2d-lite/Arbiter.h
	include/box2d-lite/Body.h
	include/box2d-lite/Joint.h
	include/box2d-lite/MathUtils.h
	include/box2d-lite/World.h)

find_package(OpenGL REQUIRED)

add_library(box2d-lite STATIC ${SOURCE_FILES} ${INCLUDE_FILES})
target_include_directories(box2d-lite PUBLIC external/glfw/include include/box2d-lite ${OPENGL_INCLUDE_DIR})

set(imgui_SOURCES
	extern/imgui/imgui.cpp
	extern/imgui/imgui_draw.cpp
	extern/imgui/imgui_widgets.cpp
	extern/imgui/examples/imgui_impl_glfw.cpp
	extern/imgui/examples/imgui_impl_opengl2.cpp)

add_subdirectory(extern/glfw)
add_executable(samples samples/main.cpp ${imgui_SOURCES})
target_include_directories(samples PUBLIC extern/imgui)
target_link_libraries(samples PUBLIC box2d-lite glfw ${GLFW_LIBRARIES} ${OPENGL_LIBRARIES} ${OPENGL_glu_LIBRARY})
