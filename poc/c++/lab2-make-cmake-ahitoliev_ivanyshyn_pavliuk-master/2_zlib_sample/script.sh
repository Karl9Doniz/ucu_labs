#!/bin/bash

SOURCE_DIR="${1:-$(pwd)}"

MAIN_SRC="$SOURCE_DIR/examples"
LIBRARY_SRC="$SOURCE_DIR/library"

BUILD_DIR="$SOURCE_DIR/build"
OBJ_BUILD_DIR="$BUILD_DIR/obj"
LIB_BUILD_DIR="$BUILD_DIR/lib"

rm -rf "$BUILD_DIR"

mkdir -p "$BUILD_DIR"
mkdir -p "$OBJ_BUILD_DIR"
mkdir -p "$LIB_BUILD_DIR"

for file in "$LIBRARY_SRC"/*.c; do
    filename=$(basename "$file")
    object_file="$OBJ_BUILD_DIR/${filename%.c}.o"
    gcc -c -fPIC -O3 -o "$object_file" "$file"
done

# Create a library (static archive)
ar rcs "$BUILD_DIR/lib/lib_static.a" "$OBJ_BUILD_DIR"/*.o

gcc -shared -fPIC -o "$BUILD_DIR/lib/lib_dynamic.so" "$OBJ_BUILD_DIR"/*.o

gcc -c -fPIC -O3 -o "$OBJ_BUILD_DIR/example.o" "$MAIN_SRC/example.c"
gcc -c -fPIC -O3 -o "$OBJ_BUILD_DIR/minigzip.o" "$MAIN_SRC/minigzip.c"

gcc "$OBJ_BUILD_DIR/example.o" -o "$LIB_BUILD_DIR/example_static" -l_static -L"$LIB_BUILD_DIR"
gcc "$OBJ_BUILD_DIR/minigzip.o" -o "$LIB_BUILD_DIR/minigzip_static" -l_static -L"$LIB_BUILD_DIR"

gcc "$OBJ_BUILD_DIR/example.o" -o "$LIB_BUILD_DIR/example_dynamic" "$BUILD_DIR/lib/lib_dynamic.so"
gcc "$OBJ_BUILD_DIR/minigzip.o" -o "$LIB_BUILD_DIR/minigzip_dynamic" "$BUILD_DIR/lib/lib_dynamic.so"