cmake_minimum_required(VERSION 3.16)
project(cmake_mystring CXX)

set(CMAKE_CXX_STANDARD 17)


add_executable(example_dynamic main.cpp)
target_link_libraries(example_dynamic functions)
target_include_directories(example_dynamic PRIVATE ${CMAKE_SOURCE_DIR}/library)

add_executable(example_static main.cpp)
target_link_libraries(example_static functions_static)
target_include_directories(example_static PRIVATE ${CMAKE_SOURCE_DIR}/library)
