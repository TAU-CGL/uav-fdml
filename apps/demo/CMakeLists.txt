if (WITH_VISUALIZATION)
    project(demo)

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
    find_package(Boost COMPONENTS container REQUIRED)
    find_package(GLEW REQUIRED)
    find_package(Lua REQUIRED)
    find_package(glm CONFIG REQUIRED)
    find_package(assimp REQUIRED)
    find_package(CGAL REQUIRED)
    find_package(OpenMP REQUIRED)
    include_directories(${GLEW_INCLUDE_DIRS} ${LUA_INCLUDE_DIR})

    if(APPLE)
        include_directories("/opt/homebrew/opt/libomp/include")
        link_directories("/opt/homebrew/opt/libomp/lib")
    endif()

    include_directories(${CMAKE_SOURCE_DIR}/third-party/LightEngine3/le3/include)
    include_directories(${PROJECT_SOURCE_DIR}/include)

    if (NOT DEFINED LUA_LIBRARIES)
        set(LUA_LIBRARIES lua::lua)
    endif()

    add_executable(${PROJECT_NAME} 
        src/main.cpp
        src/demo.cpp
    )
    target_link_libraries(${PROJECT_NAME} PRIVATE le3 fmt::fmt ${Boost_LIBRARIES} glm::glm assimp::assimp ${LUA_LIBRARIES} CGAL::CGAL OpenMP::OpenMP_CXX)
    target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
endif()