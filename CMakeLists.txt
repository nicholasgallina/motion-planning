cmake_minimum_required(VERSION 3.10)
project(MotionPlanning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(motion-planning
    src/main.cpp
    src/collision.cpp
    src/astar.cpp
)

target_include_directories(motion-planning PRIVATE include)

find_package(raylib 5.0 QUIET)
if (NOT raylib_FOUND)
    find_package(raylib 4.0 REQUIRED)
endif()

target_link_libraries(motion-planning raylib)

if(APPLE)
    target_link_libraries(motion-planning "-framework IOKit" "-framework Cocoa" "-framework OpenGL")
endif()