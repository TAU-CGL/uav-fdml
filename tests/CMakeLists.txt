project(tests)

find_package(fmt REQUIRED)
find_package(Boost COMPONENTS container REQUIRED)
find_package(GTest REQUIRED)

enable_testing()
file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_executable(tests ${SRC_FILES})
target_link_libraries(tests PRIVATE fmt::fmt ${Boost_LIBRARIES} GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(tests)