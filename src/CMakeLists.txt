cmake_minimum_required(VERSION 3.8)

set(SOURCE_FILES
	Arbiter.cpp
	Body.cpp
	Collide.cpp
	Joint.cpp
	World.cpp)

set(HEADER_FILES
	../include/box2d-lite/Arbiter.h
	../include/box2d-lite/Body.h
	../include/box2d-lite/Joint.h
	../include/box2d-lite/MathUtils.h
	../include/box2d-lite/World.h)

add_library(box2d-lite STATIC ${SOURCE_FILES} ${INCLUDE_FILES})
target_include_directories(box2d-lite PUBLIC include/box2d-lite)