cmake_minimum_required(VERSION 3.14)
project(hpipm_catkin)

# # Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_qp_solver REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(blasfeo REQUIRED)
find_package(OpenMP REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)




message(STATUS "BLASFEO_PATH: " ${BLASFEO_PATH})


# HPIPM Settings
set(BLASFEO_PATH ${BLASFEO_PATH} CACHE STRING "BLASFEO installation path" FORCE)
set(BLASFEO_INCLUDE_DIR ${BLASFEO_PATH}/include CACHE STRING "Path to BLASFEO header files." FORCE)
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(HPIPM_TESTING OFF CACHE BOOL "Examples enabled")

# Use the planner dependency snapshot bundled with this experiment.
add_subdirectory(vendor ${CMAKE_CURRENT_BINARY_DIR}/build)


# ##########
# # Build ##
# ##########
include_directories(
  include
  blasfeo
)

# Hpipm interface
add_library(${PROJECT_NAME}
  src/HpipmInterface.cpp
  src/HpipmInterfaceSettings.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ocs2_core
  ocs2_oc
  blasfeo
)
target_link_libraries(${PROJECT_NAME}
  hpipm
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
  add_clang_tooling(
    TARGETS
    ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Testing ##
#############

# if(BUILD_TESTING)

# # Include linting tests
# find_package(ament_cmake_gtest REQUIRED)

# ament_add_gtest(test_${PROJECT_NAME}
#   test/testHpipmInterface.cpp
# )
# ament_target_dependencies(${PROJECT_NAME}
#   ocs2_core
#   ocs2_oc
#   blasfeo
# )
# target_link_libraries(${PROJECT_NAME}
#   hpipm
# )

# endif(BUILD_TESTING)

#############
## Install ##
#############
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(hpipm)
ament_export_libraries(${PROJECT_NAME})

ament_package()
