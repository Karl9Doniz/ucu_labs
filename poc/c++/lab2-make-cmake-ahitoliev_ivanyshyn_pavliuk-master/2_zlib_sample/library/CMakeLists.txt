cmake_minimum_required(VERSION 3.16)
project(cmake_2_zlib_sample C)

set(CMAKE_C_STANDARD 11)

add_compile_options(-Wno-error=implicit-function-declaration)

SET(string_LIBRARY_SOURCE adler32.c compress.c crc32.c crc32.h deflate.c deflate.h gzclose.c gzguts.h
        gzlib.c gzread.c gzwrite.c infback.c inffast.c inffast.h inffixed.h inflate.c inflate.h inftrees.c
        inftrees.h trees.c trees.h uncompr.c zconf.h zlib.h zutil.c zutil.h)

if(MSVC)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()

add_library(functions SHARED ${string_LIBRARY_SOURCE})

add_library(functions_static STATIC ${string_LIBRARY_SOURCE})
