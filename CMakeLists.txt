cmake_minimum_required(VERSION 3.16)
project(bitmap_converter VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_executable(bitmap_converter
    src/main.cpp
    src/mainwindow.cpp
    src/converter.cpp
    resources/mainwindow.ui
)

target_link_libraries(bitmap_converter PRIVATE Qt6::Widgets)