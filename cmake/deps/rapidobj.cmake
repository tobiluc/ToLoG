if (NOT TARGET rapidobj::rapidobj)
    FetchContent_Declare(rapidobj
        GIT_REPOSITORY  https://github.com/cgg-bern/rapidobj-double-precision.git
        GIT_TAG         double-precision
        SOURCE_DIR "${EXTERNAL_DIR}/rapidobj"
    )
    FetchContent_MakeAvailable(rapidobj)
endif()
