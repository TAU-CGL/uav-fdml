project(experiments)

# Common for all experiments
    if(APPLE)
        set(CMAKE_C_COMPILER clang)
        set(CMAKE_CXX_COMPILER clang++)

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

    find_package(fmt REQUIRED)
    find_package(Boost COMPONENTS container program_options REQUIRED)
    find_package(GLEW REQUIRED)
    find_package(Lua REQUIRED)
    find_package(glm CONFIG REQUIRED)
    find_package(assimp REQUIRED)
    find_package(CGAL REQUIRED)
    find_package(OpenMP REQUIRED)
    find_package(nlohmann_json REQUIRED)
    include_directories(${GLEW_INCLUDE_DIRS} ${LUA_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/include)
    if(APPLE)
        include_directories("/opt/homebrew/opt/libomp/include")
        link_directories("/opt/homebrew/opt/libomp/lib")
    endif()

    include_directories(${CMAKE_SOURCE_DIR}/third-party/LightEngine3/le3/include)
    include_directories(${PROJECT_SOURCE_DIR}/include)

    if (NOT DEFINED LUA_LIBRARIES)
        set(LUA_LIBRARIES lua::lua)
    endif()

    set(EXPERIMENT_LINKS le3 fmt::fmt ${Boost_LIBRARIES} glm::glm assimp::assimp ${LUA_LIBRARIES} CGAL::CGAL OpenMP::OpenMP_CXX nlohmann_json::nlohmann_json)

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
