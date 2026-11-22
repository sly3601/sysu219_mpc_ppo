cmake_minimum_required(VERSION 3.14)
project(blasfeo_catkin)


set(CMAKE_ASM_COMPILER "${CMAKE_C_COMPILER}")

find_package(ament_cmake REQUIRED)





# BLASFEO Settings
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(TARGET X64_INTEL_HASWELL CACHE STRING "Target architecture" FORCE)
set(BLASFEO_EXAMPLES OFF CACHE BOOL "Examples enabled")
set(BLASFEO_TESTING OFF CACHE BOOL "Testing disabled")
set(BLASFEO_CROSSCOMPILING ON CACHE BOOL "Crosscompiling enabled")

# Use the planner dependency snapshot bundled with this experiment.
add_subdirectory(vendor ${CMAKE_CURRENT_BINARY_DIR}/build)


# Propagate dependencies
ament_export_libraries(blasfeo)

ament_package()
