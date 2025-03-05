cmake_minimum_required(VERSION 3.16)
project(bitmap_converter VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

# Define source files
set(SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/converter.cpp
    src/pathoptimizer.cpp
    src/svgwriter.cpp
    src/colorhandler.cpp
    src/geometryutils.cpp
)

# Define header files
set(HEADERS
    src/mainwindow.h
    src/converter.h
    src/pathoptimizer.h
    src/svgwriter.h
    src/colorhandler.h
    src/geometryutils.h
)

# UI files
set(UI_FILES
    resources/mainwindow.ui
)

add_executable(bitmap_converter
    ${SOURCES}
    ${HEADERS}
    ${UI_FILES}
)

target_link_libraries(bitmap_converter PRIVATE Qt6::Widgets)

# Set include directories
target_include_directories(bitmap_converter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Add output directory for binary
set_target_properties(bitmap_converter PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Install
install(TARGETS bitmap_converter
    RUNTIME DESTINATION bin
)