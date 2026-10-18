cmake_minimum_required(VERSION 3.10)
project(bezier_pure_pursuit LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/utf-8)
endif()

add_library(pure_pursuit src/bezier.cpp src/pure_pursuit.cpp)
target_include_directories(pure_pursuit PUBLIC include)

add_executable(example examples/main.cpp)
target_link_libraries(example PRIVATE pure_pursuit)

enable_testing()
add_executable(algorithm_tests tests/algorithm_tests.cpp)
target_link_libraries(algorithm_tests PRIVATE pure_pursuit)
add_test(NAME algorithm_tests COMMAND algorithm_tests)

