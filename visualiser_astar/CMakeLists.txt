cmake_minimum_required(VERSION 3.16)
project(PathVisualizer)

set(CMAKE_AUTOMOC ON)

set(CMAKE_CXX_STANDARD 17)
set(OSMIUM_INCLUDE_DIRS "../libs/libosmium/include")
include_directories(${OSMIUM_INCLUDE_DIRS})

find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(ZLIB REQUIRED)
find_package(EXPAT REQUIRED)
find_package(BZip2 REQUIRED)
find_package(Threads REQUIRED)

add_executable(PathVisualizer
    main.cpp
    mainwindow.cpp
    graph.cpp
)

target_include_directories(PathVisualizer PRIVATE ${CMAKE_SOURCE_DIR})

target_link_libraries(PathVisualizer
  Qt6::Widgets
  ZLIB::ZLIB
  EXPAT::EXPAT
  BZip2::BZip2
  Threads::Threads
)