cmake_minimum_required(VERSION 3.16)
project(cmake_2_zlib_sample C)

set(CMAKE_C_STANDARD 11)

if(APPLE)
    add_compile_options(-Wno-error=implicit-function-declaration)
endif()

# Add the executable and link libraries
add_executable(example_dynamic example.c)
target_link_libraries(example_dynamic functions)
target_include_directories(example_dynamic PRIVATE ${CMAKE_SOURCE_DIR}/library)

add_executable(example_minigzip_dynamic minigzip.c)
target_link_libraries(example_minigzip_dynamic functions)
target_include_directories(example_minigzip_dynamic PRIVATE ${CMAKE_SOURCE_DIR}/library)

# Add another executable and link static libraries
add_executable(example_static example.c)
target_link_libraries(example_static functions_static)
target_include_directories(example_static PRIVATE ${CMAKE_SOURCE_DIR}/library)

add_executable(example_minigzip_static minigzip.c)
target_link_libraries(example_minigzip_static functions_static)
target_include_directories(example_minigzip_static PRIVATE ${CMAKE_SOURCE_DIR}/library)
