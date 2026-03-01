include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED TRUE)
set(EXTERNAL_DIR "${PROJECT_SOURCE_DIR}/external")

include("${PROJECT_SOURCE_DIR}/cmake/deps/rapidobj.cmake")
include("${PROJECT_SOURCE_DIR}/cmake/deps/openmp.cmake")
