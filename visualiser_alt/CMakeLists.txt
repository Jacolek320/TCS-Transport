cmake_minimum_required(VERSION 3.16)
project(ALT VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets Gui)

add_executable(ALT main.cpp)
target_link_libraries(ALT PRIVATE Qt6::Core Qt6::Widgets Qt6::Gui)

# Optimization flag for performance benchmarking and fluid rendering
if(MSVC)
    target_compile_options(ALT PRIVATE /O2 /W4)
else()
    target_compile_options(ALT PRIVATE -O3 -Wall -Wextra)
endif()