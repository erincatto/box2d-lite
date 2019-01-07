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
target_include_directories(box2d-lite PUBLIC include/box2d-lite ext/glut ${OPENGL_INCLUDE_DIR})

find_package(OpenGL REQUIRED)
add_executable(samples samples/main.cpp)
target_link_libraries(samples PUBLIC box2d-lite ${CMAKE_CURRENT_SOURCE_DIR}/ext/glut/glut32.lib ${OPENGL_LIBRARIES})
target_include_directories(samples PUBLIC ext/glut ${OPENGL_INCLUDE_DIR})
