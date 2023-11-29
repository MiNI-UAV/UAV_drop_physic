cmake_minimum_required(VERSION 3.5)
project(drop)

add_subdirectory(lib/UAV_common)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
set(CMAKE_CXX_STANDARD 20)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

file(GLOB SOURCES ${SOURCE_DIR}/*.cpp)

include_directories(${INCLUDE_DIR})
link_directories(${LIB_DIR})
link_directories(${CMAKE_SOURCE_DIR}/lib/UAV_common)

link_directories("/usr/lib/x86_64-linux-gnu")
link_directories("/usr/local/lib")
link_directories("/usr/local/include")

add_executable(drop ${SOURCES})
set_property(TARGET drop PROPERTY CXX_STANDARD 20)
target_include_directories(drop PUBLIC include)
target_compile_features(drop PUBLIC cxx_std_20)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(drop Eigen3::Eigen)
find_package(cppzmq)
target_link_libraries(drop cppzmq)
find_package(cxxopts)
target_link_libraries(drop cxxopts::cxxopts)
target_link_libraries(drop common) 
target_include_directories(drop PRIVATE ${CMAKE_SOURCE_DIR}/lib/UAV_common/header)


enable_testing()

add_executable(integration_test tests/integration_test.cpp)
target_link_libraries(integration_test pthread)

add_dependencies(integration_test drop)
target_link_libraries(integration_test gtest gtest_main cppzmq Eigen3::Eigen)
add_test(NAME integration_test COMMAND integration_test)