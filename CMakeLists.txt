cmake_minimum_required(VERSION 3.16)
project(geoproc)

# Check Interprocedural Optimization (Link Time Optimization) support
include(CheckIPOSupported)
check_ipo_supported(RESULT is_ipo_supported OUTPUT error)

# Cache variables
set(ENABLE_STRICT_COMPILER_OPTIONS ON CACHE BOOL "Enable strict compile options")
set(ENABLE_ADDRESS_SANATIZER_IN_DEBUG_BUILDS OFF CACHE BOOL "Enable address sanatizer in debug builds")
set(ENABLE_NATIVE_ARCHITECTURE_OPTIMIZATIONS ON CACHE BOOL "Enable native architecture optimizations")

# Libraries
add_library(geoproc_lib
    "include/aabb.hpp"
    "include/bvh.hpp"
    "include/stl_io.hpp"
    "include/vec3.hpp"

    "src/bvh.cpp"
    "src/stl_io.cpp"
    "src/vec3.cpp"
)
target_compile_features(geoproc_lib PUBLIC cxx_std_20)
set_target_properties(geoproc_lib PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(geoproc_lib PUBLIC "include")

# Enable Inter Procedural Optimization (AKA Link Time Optimization) if supported
if(is_ipo_supported)
    set_target_properties(geoproc_lib PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
    message(WARNING "IPO is not supported: ${error}, performance may be suboptimal")
endif()

# Enable strict compiler options if requested
if(ENABLE_STRICT_COMPILER_OPTIONS)
    if(CMAKE_CXX_COMPILER_FRONTEND_VARIANT MATCHES "GNU")
        target_compile_options(geoproc_lib PUBLIC -Wall -Wextra -Wpedantic -Werror)
    elseif(CMAKE_CXX_COMPILER_FRONTEND_VARIANT MATCHES "MSVC")
        target_compile_options(geoproc_lib PUBLIC /W4 /WX)
    else()
        message(WARNING "Unknown compiler frontend variant: ${CMAKE_CXX_COMPILER_FRONTEND_VARIANT}")
    endif()
endif()

# Enable native architecture optimizations if requested
if(ENABLE_NATIVE_ARCHITECTURE_OPTIMIZATIONS)
    if(CMAKE_CXX_COMPILER_FRONTEND_VARIANT MATCHES "GNU")
        target_compile_options(geoproc_lib PUBLIC -march=native)
    elseif(CMAKE_CXX_COMPILER_FRONTEND_VARIANT MATCHES "MSVC")
        target_compile_options(geoproc_lib PUBLIC /arch:native)
    else()
        message(WARNING "Unknown compiler frontend variant: ${CMAKE_CXX_COMPILER_FRONTEND_VARIANT}")
    endif()
endif()

# Enable address sanatizer in debug builds if requested
if(ENABLE_ADDRESS_SANATIZER_IN_DEBUG_BUILDS)
    if(CMAKE_CXX_COMPILER_FRONTEND_VARIANT MATCHES "GNU")
        target_compile_options(geoproc_lib PUBLIC $<$<CONFIG:Debug>:-fsanitize=address>)
        target_link_options(geoproc_lib PUBLIC $<$<CONFIG:Debug>:-fsanitize=address>)
    elseif(CMAKE_CXX_COMPILER_FRONTEND_VARIANT MATCHES "MSVC")
        target_compile_options(geoproc_lib PUBLIC $<$<CONFIG:Debug>:/fsanitize=address>)
        target_link_options(geoproc_lib PUBLIC $<$<CONFIG:Debug>:/fsanitize=address>)
    else()
        message(WARNING "Unknown compiler frontend variant: ${CMAKE_CXX_COMPILER_FRONTEND_VARIANT}")
    endif()
endif()

# Applications
add_executable(plane_clipping2 "apps/plane_clipping2.cpp")
target_link_libraries(plane_clipping2 PRIVATE geoproc_lib)

add_executable(bvh_playground "apps/bvh_playground.cpp")
target_link_libraries(bvh_playground PRIVATE geoproc_lib)
