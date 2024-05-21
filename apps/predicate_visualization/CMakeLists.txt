if (WITH_VISUALIZATION)
    project(predicate_visualization)

    find_package(fmt REQUIRED)
    find_package(Boost COMPONENTS container REQUIRED)
    find_package(GLEW REQUIRED)
    find_package(Lua REQUIRED)
    find_package(glm CONFIG REQUIRED)
    find_package(assimp REQUIRED)
    find_package(CGAL REQUIRED)
    include_directories(${GLEW_INCLUDE_DIRS} ${LUA_INCLUDE_DIR})

    include_directories(${CMAKE_SOURCE_DIR}/third-party/LightEngine3/le3/include)
    include_directories(${PROJECT_SOURCE_DIR}/include)

    if (NOT DEFINED LUA_LIBRARIES)
        set(LUA_LIBRARIES lua::lua)
    endif()

    add_executable(${PROJECT_NAME} 
        src/main.cpp 
    )
    target_link_libraries(${PROJECT_NAME} PRIVATE le3 fmt::fmt ${Boost_LIBRARIES} glm::glm assimp::assimp ${LUA_LIBRARIES} CGAL::CGAL)
    target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
endif()