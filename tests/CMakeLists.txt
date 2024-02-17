project(tests)

find_package(fmt REQUIRED)
find_package(Boost COMPONENTS container REQUIRED)
find_package(GTest REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/include)
set(TEST_LINKS fmt::fmt ${Boost_LIBRARIES} GTest::gtest)

file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
set (TESTS)
foreach (FILEPATH ${SRC_FILES})
    get_filename_component(FILEPATH_WE ${FILEPATH} NAME_WE)
    list(APPEND TESTS ${FILEPATH_WE})
endforeach()

foreach(TEST ${TESTS})
    enable_testing()
    add_executable(${TEST} "src/${TEST}.cpp")
    target_link_libraries(${TEST} PRIVATE ${TEST_LINKS})
    # add_test(NAME ${TEST} COMMAND ${CMAKE_BINARY_DIR}/${TEST})

    include(GoogleTest)
    gtest_discover_tests(${TEST})
endforeach()
