#
# External dependencies
#

if (NOT ${OpenCV_VERSION})
  set(OpenCV_VERSION 3.0)
endif ()
find_package(OpenCV ${OpenCV_VERSION} REQUIRED core imgproc calib3d highgui)
find_package(Eigen)
find_package(Sophus)
#
# Library name and options
#

# Target name
set(target vision_core)

# Exit here if required dependencies are not met
message(STATUS "Lib ${target}")

# Set API export file and macro
#string(TOUPPER ${target} target_upper)
#set(feature_file "include/${target}/${target}_features.h")
#set(export_file  "include/${target}/${target}_api.h")
#set(export_macro "${target_upper}_API")
#
set(include_path "${CMAKE_CURRENT_SOURCE_DIR}/include/${target}")
set(local_include_path "${CMAKE_CURRENT_SOURCE_DIR}/include_local/${target}")
set(source_path "${CMAKE_CURRENT_SOURCE_DIR}/src")


set(local_include_files
    ${lib_header}
    )

set(headers
    ${include_path}/types.hpp
    ${include_path}/utils.hpp
    ${include_path}/config_helper.hpp
    ${include_path}/math.hpp
    ${local_include_files}
    )

set(sources )

# Group source files
set(header_group "Header Files (API)")
set(source_group "Source Files")
source_group_by_path(${include_path} "\\\\.h$|\\\\.hpp$"
    ${header_group} ${headers})
source_group_by_path(${source_path} "\\\\.cpp$|\\\\.c$|\\\\.h$|\\\\.hpp$"
    ${source_group} ${sources})

#
# Create library
##
include_directories(
    include_local
    include
)

# Build library
add_library(${target} INTERFACE
    )
target_sources(${target} INTERFACE ${headers})

# Create namespaced alias
add_library(${META_PROJECT_NAME}::${target} ALIAS ${target})

# Export library for downstream projects
#export(TARGETS ${target} NAMESPACE ${META_PROJECT_NAME}:: FILE ${PROJECT_BINARY_DIR}/cmake/${target}/${target}-export.cmake)

## Create feature detection header
## Compilers: https://cmake.org/cmake/help/v3.1/variable/CMAKE_LANG_COMPILER_ID.html#variable:CMAKE_%3CLANG%3E_COMPILER_ID
## Feature: https://cmake.org/cmake/help/v3.1/prop_gbl/CMAKE_CXX_KNOWN_FEATURES.html
#write_compiler_detection_header(
#    FILE ${feature_file}
#    PREFIX ${target_upper}
#    COMPILERS AppleClang Clang GNU MSVC
#    FEATURES cxx_alignas cxx_alignof cxx_constexpr cxx_final cxx_noexcept cxx_nullptr cxx_sizeof_member cxx_thread_local
#)
#
## Create API export header
#generate_export_header(${target}
#    EXPORT_FILE_NAME  ${export_file}
#    EXPORT_MACRO_NAME ${export_macro}
#)

#
# Include directories
#


target_include_directories(${target}
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include_local
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${DEFAULT_INCLUDE_DIRECTORIES}
    )

#
# Project options
#




#
# Libraries
#

target_link_libraries(${target}
    PRIVATE

    PUBLIC

    INTERFACE
    ${DEFAULT_LIBRARIES}
    Sophus
    utilities
    ${OpenCV_LIBRARIES}
    )

