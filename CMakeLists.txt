cmake_minimum_required(VERSION 3.20)

project(robotransforms_core VERSION 0.2.0 LANGUAGES CXX)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robotransforms/version.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/robotransforms/version.hpp
    @ONLY
)

add_library(robotransforms_euclidean INTERFACE)
target_compile_features(robotransforms_euclidean INTERFACE cxx_std_20)
target_include_directories(robotransforms_euclidean INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/include
)

enable_testing()

add_executable(euclidean_test
    tests/euclidean_test.cpp
)
target_link_libraries(euclidean_test PRIVATE robotransforms_euclidean)

add_test(NAME euclidean_test COMMAND euclidean_test)
