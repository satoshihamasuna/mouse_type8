# Set the minimum version of CMake that can be used
cmake_minimum_required(VERSION 3.20)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Project definition
project(mouse_type8)

# Enable CMake support for ASM and C/CPP languages
enable_language(C CXX ASM)

set(BUILD_UNIT_0_NAME mouse_type8)

# Select the target machine. Build presets set this explicitly; A keeps
# legacy command-line builds compatible with the machine developed so far.
set(MOUSE_TARGET "A" CACHE STRING "Target machine (A or B)")
set_property(CACHE MOUSE_TARGET PROPERTY STRINGS A B)
string(TOUPPER "${MOUSE_TARGET}" MOUSE_TARGET_UPPER)

# Create an executable object type
add_executable(${BUILD_UNIT_0_NAME})

if(MOUSE_TARGET_UPPER STREQUAL "A")
  target_compile_definitions(${BUILD_UNIT_0_NAME} PRIVATE MOUSE_A)
elseif(MOUSE_TARGET_UPPER STREQUAL "B")
  target_compile_definitions(${BUILD_UNIT_0_NAME} PRIVATE MOUSE_B)
else()
  message(FATAL_ERROR "MOUSE_TARGET must be A or B (received: ${MOUSE_TARGET})")
endif()

message(STATUS "Building for MOUSE_${MOUSE_TARGET_UPPER}")

# STM32CubeIDE for Visual Studio Code managed CMake file
include(cmake/vscode_generated.cmake)

# Compiler options
target_compile_options(${BUILD_UNIT_0_NAME} PRIVATE
  # User defined compiler options
)

# Linker options
target_link_options(${BUILD_UNIT_0_NAME} PRIVATE 
  # User defined linker options
)

# Add sources to executable/library
target_sources(${BUILD_UNIT_0_NAME} PRIVATE
  # User defined sources
)

# Add include directories
target_include_directories(${BUILD_UNIT_0_NAME} PRIVATE
  "${CMAKE_SOURCE_DIR}"
)
