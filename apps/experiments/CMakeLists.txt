project(experiments)

# Common for all experiments
find_package(fmt REQUIRED)
find_package(Boost COMPONENTS container program_options REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/include)
set(EXPERIMENT_LINKS fmt::fmt ${Boost_LIBRARIES})

# Experiments are added here. Only a single cpp file supported (but unlimited # of headers)
set (EXPERIMENTS 
    ex_kdtree_construct
)

# Add executable foreach experiment
foreach(EXPERIMENT ${EXPERIMENTS})
    add_executable(${EXPERIMENT} "src/${EXPERIMENT}.cpp")
    target_link_libraries(${EXPERIMENT} PRIVATE ${EXPERIMENT_LINKS})
endforeach()
