cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(offboard)

include_directories(
    include/mavlink_interface
)

add_executable(offboard
    src/driver.cpp
    src/aux.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(offboard
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(offboard PRIVATE -Wall -Wextra -O3)
else()
    add_compile_options(offboard PRIVATE -WX -W2)
endif()

if(MINGW OR MSVC)
    # For M_PI
    add_target_definitions(offboard "-D_USE_MATH_DEFINES")
endif()