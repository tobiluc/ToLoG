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
# Add Library
#================================================
add_library("${PROJECT_NAME}" STATIC
    src/ToLoG/predicates/predicates.c
    src/ToLoG/predicates/predicates_wrapper.cpp
    src/ToLoG/predicates/predicates_wrapper.hpp

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

    src/ToLoG/unstable/SimplexSolver.hpp
    src/ToLoG/optimization/Sparse.hpp
    src/ToLoG/unstable/LinearProgram.hpp
    src/ToLoG/unstable/SimplexSolver.cpp
    src/ToLoG/io/obj_writer.hpp
    src/ToLoG/io/happly.hpp
    src/ToLoG/io/ply_writer.hpp
    src/ToLoG/utils/indices.hpp

    src/ToLoG/detail/Traits.inl.hpp
    src/ToLoG/detail/squared_distance.inl.hpp
    src/ToLoG/detail/barycentric.inl.hpp
    src/ToLoG/detail/aabb.inl.hpp
    src/ToLoG/detail/centroid.inl.hpp
    src/ToLoG/Traits_fwd.hpp
    src/ToLoG/detail/vector.inl.hpp
    src/ToLoG/Ray.hpp
    src/ToLoG/math/polynomial_roots.hpp
    src/ToLoG/detail/triangle.inl.hpp
    src/ToLoG/math/winding_number.hpp
    src/ToLoG/mesh/delauney_triangulation.hpp
    src/ToLoG/detail/intersects.inl.hpp
    src/ToLoG/mesh/mesh_types.hpp
    src/ToLoG/predicates/derived_predicates.hpp







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
#if(OpenMP_CXX_FOUND)
#    target_compile_options(${PROJECT_NAME} PUBLIC ${OpenMP_CXX_FLAGS})
#endif()

#=========================
# Link Libraries
#=========================
include_directories (${INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
OpenMP::OpenMP_CXX
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

#=========================
# Testing
#=========================
set(TOLOG_BUILD_UNIT_TESTS true CACHE BOOL "Whether to build the unit tests.")
if (TOLOG_BUILD_UNIT_TESTS)
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
