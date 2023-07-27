if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    cmake_minimum_required(VERSION 3.3)
    project(algo)

    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/QuadProgpp/src)

    include_directories(/opt/homebrew/Cellar/eigen/3.4.0_1/include)

    find_package(Eigen3 3.4 REQUIRED NO_MODULE)

    aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src DIR_SRCS)

    add_library(algolib SHARED ${DIR_SRCS})

    target_include_directories(algolib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/QuadProgpp/src
    )

    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/QuadProgpp/src)
    target_link_libraries(algolib Eigen3::Eigen ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/QuadProgpp/src/libquadprog.a)



    add_executable(algo ${DIR_SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

    target_link_libraries(algo Eigen3::Eigen libquadprog.a)
else()

    cmake_minimum_required(VERSION 3.3)
    project(algo)

    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/QuadProgpp/src)

    # include_directories(/usr/local/include/QuadProg++)

    find_package(Eigen3 3.3 REQUIRED NO_MODULE)

    aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src DIR_SRCS)

    add_library(algolib SHARED ${DIR_SRCS})

    target_include_directories(algolib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/QuadProgpp/src
    )

    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/QuadProgpp/src)
    target_link_libraries(algolib Eigen3::Eigen ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/QuadProgpp/src/libquadprog.a)



    add_executable(algo ${DIR_SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

    target_link_libraries(algo Eigen3::Eigen libquadprog.a)
endif()


