if(APPLE)
    set(OpenMP_ROOT "/opt/homebrew/opt/libomp") # this is a bit of a hack
endif()
find_package(OpenMP COMPONENTS CXX REQUIRED)