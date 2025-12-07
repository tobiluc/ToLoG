
cmake_minimum_required(VERSION 3.4)

add_executable (ToLoG_cli main.cc)
target_link_libraries(ToLoG_cli PRIVATE
    CLI11::CLI11
    ToLoG)

target_compile_options(ToLoG_cli PUBLIC
    $<$<CXX_COMPILER_ID:Clang>:-Wno-long-long>
    $<$<CXX_COMPILER_ID:GNU>:-pedantic -Wno-long-long>
    )
