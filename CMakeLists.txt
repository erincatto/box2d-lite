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

add_library(box2d-lite STATIC ${SOURCE_FILES} ${INCLUDE_FILES})
target_include_directories(box2d-lite PUBLIC include/box2d-lite)

option(BOX2D_BUILD_SAMPLES "Build the box2d-lite sample program" ON)

if (BOX2D_BUILD_SAMPLES)
find_package(OpenGL REQUIRED)

set(imgui_SOURCES
	extern/imgui/imgui.cpp
	extern/imgui/imgui_draw.cpp
	extern/imgui/imgui_widgets.cpp
	extern/imgui/examples/imgui_impl_glfw.cpp
	extern/imgui/examples/imgui_impl_opengl2.cpp)

option(BUILD_SHARED_LIBS "" OFF)
option(GLFW_BUILD_EXAMPLES "" OFF)
option(GLFW_BUILD_TESTS "" OFF)
option(GLFW_BUILD_DOCS "" OFF)
option(GLFW_INSTALL "" OFF)
option(GLFW_VULKAN_STATIC "" OFF)
if (MSVC)
    option(USE_MSVC_RUNTIME_LIBRARY_DLL "" OFF)
endif()

add_subdirectory(extern/glfw)

add_executable(samples samples/main.cpp ${imgui_SOURCES})
target_include_directories(samples PUBLIC extern/imgui ${OPENGL_INCLUDE_DIR})
target_link_libraries(samples PUBLIC box2d-lite glfw ${OPENGL_LIBRARIES} ${OPENGL_glu_LIBRARY})

set(VS_STARTUP_PROJECT samples)
endif()
