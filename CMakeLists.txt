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

add_subdirectory(external/glfw)
add_executable(samples samples/main.cpp)
# target_include_directories(samples PUBLIC external/glfw/include ${OPENGL_INCLUDE_DIR})
target_link_libraries(samples PUBLIC box2d-lite glfw ${GLFW_LIBRARIES} ${OPENGL_LIBRARIES} ${OPENGL_glu_LIBRARY})
