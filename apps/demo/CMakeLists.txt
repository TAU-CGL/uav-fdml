project(demo)

find_package(fmt REQUIRED)
find_package(Boost COMPONENTS container REQUIRED)

add_executable(demo src/main.cpp)
target_link_libraries(demo PRIVATE fmt::fmt ${Boost_LIBRARIES})
