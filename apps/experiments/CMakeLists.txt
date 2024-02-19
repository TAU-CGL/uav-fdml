project(experiments)

# Common for all experiments
find_package(fmt REQUIRED)
find_package(Boost COMPONENTS container program_options REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/include)
set(EXPERIMENT_LINKS fmt::fmt ${Boost_LIBRARIES} OpenMP::OpenMP_CXX)

file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
set (EXPERIMENTS)
foreach (FILEPATH ${SRC_FILES})
    get_filename_component(FILEPATH_WE ${FILEPATH} NAME_WE)
    list(APPEND EXPERIMENTS ${FILEPATH_WE})
endforeach()

# Add executable foreach experiment
foreach(EXPERIMENT ${EXPERIMENTS})
    add_executable(${EXPERIMENT} "src/${EXPERIMENT}.cpp")
    target_link_libraries(${EXPERIMENT} PRIVATE ${EXPERIMENT_LINKS})
endforeach()
