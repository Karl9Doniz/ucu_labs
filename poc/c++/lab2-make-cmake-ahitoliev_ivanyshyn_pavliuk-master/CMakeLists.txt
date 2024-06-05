cmake_minimum_required(VERSION 3.16)
project(FindMyStringLibrary)

find_library(
        MYSTRING_LIBRARY_STATIC
        NAMES  libfunctions_static.a
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}/mystring/b/library
)

if(MYSTRING_LIBRARY_STATIC)
    message(STATUS "MyString static library found: ${MYSTRING_LIBRARY_STATIC}")
else()
    message(FATAL_ERROR "MyString  static library not found")
endif()

find_library(
        MYSTRING_LIBRARY
        NAMES  libfunctions.dylib
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}/mystring/b/library
)

if(MYSTRING_LIBRARY)
    message(STATUS "MyString dynamic library found: ${MYSTRING_LIBRARY}")
else()
    message(FATAL_ERROR "MyString dynamic library not found")
endif()