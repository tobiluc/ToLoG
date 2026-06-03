#================================================
# Add Library
#================================================
set(TOLOG_LIBRARY_NAME "${PROJECT_NAME}")
add_library("${TOLOG_LIBRARY_NAME}" STATIC
    ToLoG/geometry/predicates/predicates.c
    ToLoG/geometry/predicates/predicates.cpp
    ToLoG/unstable/marching_cubes.cpp
    ToLoG/math/polynomial_roots.cpp
    ToLoG/mesh/PolygonMesh.cpp
)
add_library ("${TOLOG_LIBRARY_NAME}::${TOLOG_LIBRARY_NAME}" ALIAS ${TOLOG_LIBRARY_NAME})

#================================================
# Core Options
#================================================
option(TOLOG_EXACT_SINGLE_PRECISION "Use single instead of double precision for exact predicates" OFF)
if (TOLOG_EXACT_SINGLE_PRECISION)
    message(STATUS "using single precision (float) for exact predicates")
    target_compile_definitions(${TOLOG_LIBRARY_NAME} PUBLIC TOLOG_EXACT_SINGLE_PRECISION)
else ()
    message(STATUS "using double precision (double) for exact predicates")
endif ()

#=========================
# Link Libraries
#=========================
include_directories (${INCLUDE_DIRS})
target_link_libraries(${TOLOG_LIBRARY_NAME} PUBLIC
OpenMP::OpenMP_CXX
rapidobj::rapidobj
)

#=========================
# Define what to include
#=========================
target_include_directories(${TOLOG_LIBRARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

set_source_files_properties(ToLoG/geometry/predicates/predicates.c
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
# Fake successful finder run if compiling as a dependent project (Whatever that means...)
#=========================
if(NOT "${CMAKE_PROJECT_NAME}" MATCHES "ToLoG")
    set (TOLOG_FOUND true PARENT_SCOPE)
    set (TOLOG_LIBRARIES ToLoG PARENT_SCOPE)
    set (TOLOG_LIBRARY ToLoG PARENT_SCOPE)
    set (TOLOG_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src/" PARENT_SCOPE)
    set (TOLOG_LIBRARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "The directory where the ToLoG libraries can be found.")
endif()
