cmake_minimum_required(VERSION 3.11)

project(se3loc)

find_package(fmt REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/kdtree.cpp
    src/geometry.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PRIVATE fmt::fmt)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)