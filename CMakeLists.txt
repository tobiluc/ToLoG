cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
cmake_policy(VERSION 3.18.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(ToLoG VERSION 0.1.0 LANGUAGES C CXX)

# Allow big files for gnu compiler
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-mbig-obj")
endif()

#================================================
# Check if standalone or within other project
#================================================
if( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR} )
    set(TOLOG_STANDALONE_BUILD TRUE)
    message("Configuring ToLoG as standalone project...")
else()
    set(TOLOG_STANDALONE_BUILD FALSE)
    message("Configuring ToLoG inside another cmake project...")
endif()

if(TOLOG_STANDALONE_BUILD)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/Build/bin")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/Build/lib")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/Build/lib")
endif()

#================================================
# CMake Finders
#================================================
if(TOLOG_STANDALONE_BUILD)
    list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
else()
    set(_my_cmake_module_path "${CMAKE_CURRENT_LIST_DIR}/cmake")
    set(CMAKE_MODULE_PATH "${_my_cmake_module_path};${CMAKE_MODULE_PATH}" CACHE STRING "CMake module path" FORCE)
endif()

#================================================
# Add Library
#================================================
add_library("${PROJECT_NAME}" STATIC
    src/ToLoG/predicates/predicates.c
    src/ToLoG/predicates/ExactPredicates.cpp
    src/ToLoG/predicates/ExactPredicates.hpp

    src/ToLoG/Core.hpp
    src/ToLoG/AABBTree.hpp
    src/ToLoG/DSU.hpp
    src/ToLoG/HashMap.hpp
    src/ToLoG/Heap.hpp

    src/ToLoG/utils/KeyValueVector.hpp
    src/ToLoG/utils/enumerate.hpp

    src/ToLoG/unstable/Octree.hpp
    src/ToLoG/unstable/Octree.cpp
    src/ToLoG/unstable/marching_cubes.cpp
    src/ToLoG/unstable/marching_cubes.hpp
    src/ToLoG/unstable/KDTree.hpp

    src/ToLoG/optimization/SimplexSolver.hpp
    src/ToLoG/optimization/Sparse.hpp
    src/ToLoG/optimization/LinearProgram.hpp
    src/ToLoG/optimization/SimplexSolver.cpp
)
add_library ("${PROJECT_NAME}::${PROJECT_NAME}" ALIAS ${PROJECT_NAME})

#=========================
# Init Fetch Content
#=========================
include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED TRUE)
set(SUBMODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/submodules")
set(EXTERNAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external")

#=========================
# OpenMP
#=========================
if(APPLE)
    set(OpenMP_ROOT "/opt/homebrew/opt/libomp") # this is a bit of a hack
endif()
find_package(OpenMP COMPONENTS CXX REQUIRED)
if(OpenMP_CXX_FOUND)
    target_compile_options(${PROJECT_NAME} PUBLIC ${OpenMP_CXX_FLAGS})
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()

#=========================
# NLohmann Json
#=========================
if (NOT TARGET nlohmann_json::nlohmann_json)
    FetchContent_Declare(
        json
        URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
    )
    FetchContent_MakeAvailable(json)
endif()

#=========================
# Link Libraries
#=========================
include_directories (${INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
OpenMP::OpenMP_CXX
nlohmann_json::nlohmann_json
)

#=========================
# Define what to include
#=========================
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/src>
)

set_source_files_properties(src/ToLoG/predicates/predicates.c
    PROPERTIES
    # make "if MSVC" work
    COMPILE_DEFINITIONS "$<$<CXX_COMPILER_ID:MSVC>:MSVC=1>"
# Shewchuk's predicates.c use some old-style C, we don't need warnings about it:
    COMPILE_FLAGS "$<$<CXX_COMPILER_ID:MSVC>:/wd4131>"
    #COMPILE_FLAGS "$<$<CXX_COMPILER_ID:GNU>:-Wno-deprecated-non-prototype>"
    COMPILE_FLAGS "$<$<CXX_COMPILER_ID:Clang>:-Wno-deprecated-non-prototype -Wno-incompatible-pointer-types-discards-qualifiers>"
    COMPILE_FLAGS "$<$<CXX_COMPILER_ID:AppleClang>:-Wno-deprecated-non-prototype -Wno-incompatible-pointer-types-discards-qualifiers>"
    LANGUAGE C
)

#================================
# Add the standalone executables
#================================
if(TOLOG_STANDALONE_BUILD)
    if(NOT TARGET CLI11::CLI11)
        FetchContent_Declare(cli11
            GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
            GIT_TAG        v2.5.0
            SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/CLI11"
            )
        FetchContent_MakeAvailable(cli11)
        message("Downloaded CLI11 to ${cli11_SOURCE_DIR}")
    endif()
   add_subdirectory(tools/ToLoG)
endif()

#=========================
# Testing
#=========================
set(TOLOG_BUILD_UNIT_TESTS true CACHE BOOL "Whether to build the unit tests.")
if (TOLOG_STANDALONE_BUILD AND TOLOG_BUILD_UNIT_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
    add_subdirectory(tests)
endif()

#=========================
# Fake successful finder run if compiling as a dependent project (Whatever that means...)
#=========================
if(NOT "${CMAKE_PROJECT_NAME}" MATCHES "ToLoG")
    set (TOLOG_FOUND true PARENT_SCOPE)
    set (TOLOG_LIBRARIES ToLoG PARENT_SCOPE)
    set (TOLOG_LIBRARY ToLoG PARENT_SCOPE)
    set (TOLOG_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src/" PARENT_SCOPE)
    set (TOLOG_LIBRARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "The directory where the ToLoG libraries can be found.")
endif()
