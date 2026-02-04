if (GTEST_FOUND)
    include(GoogleTest)

    FILE(GLOB UNITTEST_SRC
        aabbtree_tests.cpp
        core_tests.cpp
        customization_tests.cpp
        dsu_tests.cpp
        enumerate_tests.cpp
        eigen_tests.cpp
        hash_tests.cpp
        heap_tests.cpp
        keyvaluevector_tests.cpp
        octree_tests.cpp
        predicates_tests.cpp
        ray_tests.cpp
        simplex_tests.cpp
        sparse_tests.cpp
        winding_number_tests.cpp
        unittests.cpp
    )

    # Get Eigen
    if(NOT TARGET Eigen3::Eigen)
        FetchContent_Declare(Eigen3 SYSTEM
            GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
            GIT_TAG 3.4.0
            GIT_SHALLOW TRUE
            SOURCE_SUBDIR cmake
        )
        FetchContent_MakeAvailable(Eigen3)
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${eigen3_SOURCE_DIR})
        target_link_libraries(Eigen3::Eigen INTERFACE OpenMP::OpenMP_CXX)
    endif()

    # Create unittest executable
    add_executable(ToLoG-unittests ${UNITTEST_SRC})
    target_link_libraries(ToLoG-unittests
        ToLoG::ToLoG
        GTest::GTest
        GTest::Main
        Eigen3::Eigen
    )

    if ( NOT WIN32)
      target_link_libraries(ToLoG-unittests pthread)
    endif()

    # For the unittest we don't want the install rpath as set by acg_add_executable
    set_target_properties (ToLoG-unittests PROPERTIES  BUILD_WITH_INSTALL_RPATH 0 )

    # Set output directory to ${BINARY_DIR}/Build
    set (OUTPUT_DIR "${CMAKE_BINARY_DIR}/Build/${ACG_PROJECT_BINDIR}")
    set_target_properties(ToLoG-unittests PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR})

    foreach(CONFIG ${CMAKE_CONFIGURATION_TYPES})
      string(TOUPPER ${CONFIG} UPCONFIG)
      set_target_properties(ToLoG-unittests PROPERTIES RUNTIME_OUTPUT_DIRECTORY_${UPCONFIG} ${OUTPUT_DIR})
    endforeach()

    if ( CMAKE_GENERATOR MATCHES "^Visual Studio 11.*" )
        target_compile_definitions( _VARIADIC_MAX=10 )
    endif()


    if ( NOT WIN32 )
      # Set compiler flags
      if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set_target_properties(ToLoG-unittests PROPERTIES COMPILE_FLAGS "-g -Wno-long-long")
      else()
        set_target_properties(ToLoG-unittests PROPERTIES COMPILE_FLAGS "-g -pedantic -Wno-long-long")
      endif()
    else()
      # Set compiler flags
      set_target_properties(ToLoG-unittests PROPERTIES COMPILE_FLAGS "" )
    endif()

    gtest_add_tests(TARGET ToLoG-unittests
        EXTRA_ARGS "--gtest_color=yes;--gtest_output=xml"
        WORKING_DIRECTORY "${OUTPUT_DIR}"
        )

else(GTEST_FOUND)
    message(STATUS "Google testing framework was not found, ToLoG-unittests disabled.")
endif(GTEST_FOUND)
