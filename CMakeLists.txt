cmake_minimum_required(VERSION 3.20)

project(robotransforms_core VERSION 0.3.0 LANGUAGES CXX)

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

set(ROBOTRANSFORMS_DIST_DIR ${CMAKE_CURRENT_BINARY_DIR}/dist)
set(ROBOTRANSFORMS_DIST_HEADER_VERSIONED ${ROBOTRANSFORMS_DIST_DIR}/robotransforms-core-${PROJECT_VERSION}.hpp)
set(ROBOTRANSFORMS_DIST_HEADER_LATEST ${ROBOTRANSFORMS_DIST_DIR}/robotransforms-core.hpp)

add_custom_command(
    OUTPUT
        ${ROBOTRANSFORMS_DIST_HEADER_VERSIONED}
        ${ROBOTRANSFORMS_DIST_HEADER_LATEST}
    COMMAND ${CMAKE_COMMAND}
        -DINPUT_VERSION_HEADER=${CMAKE_CURRENT_BINARY_DIR}/include/robotransforms/version.hpp
        -DINPUT_EUCLIDEAN_HEADER=${CMAKE_CURRENT_SOURCE_DIR}/include/robotransforms/euclidean.hpp
        -DOUTPUT_VERSIONED_HEADER=${ROBOTRANSFORMS_DIST_HEADER_VERSIONED}
        -DOUTPUT_LATEST_HEADER=${ROBOTRANSFORMS_DIST_HEADER_LATEST}
        -DPROJECT_VERSION=${PROJECT_VERSION}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateDistHeader.cmake
    DEPENDS
        ${CMAKE_CURRENT_BINARY_DIR}/include/robotransforms/version.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/robotransforms/euclidean.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateDistHeader.cmake
    VERBATIM
)

add_custom_target(dist_header
    DEPENDS
        ${ROBOTRANSFORMS_DIST_HEADER_VERSIONED}
        ${ROBOTRANSFORMS_DIST_HEADER_LATEST}
)

enable_testing()

add_executable(euclidean_test
    tests/euclidean_test.cpp
)
target_link_libraries(euclidean_test PRIVATE robotransforms_euclidean)

add_test(NAME euclidean_test COMMAND euclidean_test)