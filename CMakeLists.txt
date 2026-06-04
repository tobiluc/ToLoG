cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
cmake_policy(VERSION 3.18.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# Allow big files for gnu compiler
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-mbig-obj")
endif()

project(ToLoG VERSION 0.1.0 LANGUAGES CXX C)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/dependencies.cmake")

add_subdirectory(src)

#=========================
# Testing
#=========================
set(TOLOG_BUILD_UNIT_TESTS true CACHE BOOL "Whether to build the unit tests.")
if (TOLOG_BUILD_UNIT_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
    add_subdirectory(tests)
endif()
