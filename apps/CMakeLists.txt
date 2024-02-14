project(apps)

find_package(fmt REQUIRED)

add_executable(demo demo/main.cpp)
target_include_directories(demo PUBLIC ${CMAKE_SOURCE_DIR}/libs)
target_link_libraries(demo PRIVATE se3loc fmt::fmt)