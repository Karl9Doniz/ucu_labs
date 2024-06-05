cmake_minimum_required(VERSION 3.16)
project(cmake_mystring CXX)

set(CMAKE_CXX_STANDARD 17)

SET(string_LIBRARY_SOURCE mystring.cpp mystring.hpp)

if(MSVC)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()

add_library(functions SHARED ${string_LIBRARY_SOURCE})

add_library(functions_static STATIC ${string_LIBRARY_SOURCE})