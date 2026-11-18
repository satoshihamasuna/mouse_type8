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
  "Task/Src/zirconia_wall_control.cpp"
  "Task/Src/diagonal_pose.cpp"
  "Module/Src/log_stream.cpp"
  "Module/Src/remote_explore.cpp"
  "Module/Src/interrupt_timing.cpp"
  "System/Src/explore.cpp"
  "System/Src/shortest_run.cpp"
  "Params/Param_${MOUSE_TARGET_UPPER}/run_param_${MOUSE_TARGET_UPPER}.cpp"
  "Params/Param_${MOUSE_TARGET_UPPER}/turn_search/turn_search.cpp"
  "Params/Param_${MOUSE_TARGET_UPPER}/turn_300/turn_300.cpp"
  "Params/Param_${MOUSE_TARGET_UPPER}/turn_500/turn_500.cpp"
  "Params/Param_${MOUSE_TARGET_UPPER}/turn_700/turn_700.cpp"
  "Params/Param_${MOUSE_TARGET_UPPER}/turn_1000/turn_1000.cpp"
  "Params/Param_${MOUSE_TARGET_UPPER}/turn_1200/turn_1200.cpp"
  "Params/Param_${MOUSE_TARGET_UPPER}/turn_1400/turn_1400.cpp"
  "Params/Param_${MOUSE_TARGET_UPPER}/turn_1500/turn_1500.cpp"
  "Params/Param_${MOUSE_TARGET_UPPER}/turn_1600/turn_1600.cpp"
  "Params/Param_${MOUSE_TARGET_UPPER}/turn_1800/turn_1800.cpp"
  "Params/Param_${MOUSE_TARGET_UPPER}/turn_2000/turn_2000.cpp"
)

# Add include directories
target_include_directories(${BUILD_UNIT_0_NAME} PRIVATE
  "${CMAKE_SOURCE_DIR}"
)
