cmake_minimum_required(VERSION 3.16)
project(geoproc)

# Check Interprocedural Optimization (Link Time Optimization) support
include(CheckIPOSupported)
check_ipo_supported(RESULT is_ipo_supported OUTPUT error)

add_library(geoproc_lib
    "include/bvh.hpp"
    "include/vec3.hpp"

    "src/bvh.cpp"
    "src/vec3.cpp"
)
target_compile_features(geoproc_lib PUBLIC cxx_std_11)
set_target_properties(geoproc_lib PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(geoproc_lib PUBLIC "include")

# Enable IPO(aka LTO) if supported
if(is_ipo_supported)
    set_target_properties(geoproc_lib PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
    message(WARNING "IPO is not supported: ${error}")
endif()

# Applications
add_executable(plane_clipping "apps/plane_clipping.cpp")
target_link_libraries(plane_clipping PRIVATE geoproc_lib)
