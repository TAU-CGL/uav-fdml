cmake_minimum_required(VERSION 3.20)

##########################
# Default compile options
##########################

if (NOT DEFINED WITH_VISUALIZATION) 
    set (WITH_VISUALIZATION true) 
endif()

if (NOT DEFINED SKIP_PKG_RESOURCES)
    set (SKIP_PKG_RESOURCES true)
endif()

##########################

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

if (WIN32)
    add_compile_options(/bigobj)
endif ()

project(se3-localization)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include_directories("include")

if (WITH_VISUALIZATION)
    set(LE3_WITH_DEMOS false)
    add_subdirectory(third-party/LightEngine3)
endif()

############################################################
#@ Setup OpenMP & CGAL
############################################################
if(APPLE)
    # set(CMAKE_C_COMPILER clang)
    # set(CMAKE_CXX_COMPILER clang++)

    if(CMAKE_C_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp")
        set(OpenMP_C_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY omp)
    endif()

    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp")
        set(OpenMP_CXX_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY omp)
    endif()
endif()
find_package(OpenMP REQUIRED)
if(APPLE)
    include_directories("/opt/homebrew/opt/libomp/include")
    link_directories("/opt/homebrew/opt/libomp/lib")
endif()

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
find_package(CGAL REQUIRED)
############################################################

add_subdirectory(apps)

# Copy the resources directory to the build
if (NOT SKIP_PKG_RESOURCES)
if (WIN32)
else()
    add_custom_target(
        pkg-resources-tgt-fdml ALL
        COMMAND 
            ${CMAKE_SOURCE_DIR}/third-party/LightEngine3/bin/pkg-resources ${CMAKE_CURRENT_SOURCE_DIR}/resources ${CMAKE_SOURCE_DIR}/bin

        DEPENDS ${CMAKE_SOURCE_DIR}/third-party/LightEngine3/bin/pkg-resources
    )
endif()
else()
    message(WARNING "Skipping pkg resources for fdml")
endif()