#!/bin/bash

SOURCE_DIR="${1:-$(pwd)}"
LIB_NAME="${2:-mystring}"

MAIN_SRC="$SOURCE_DIR/examples/main.cpp"
MYSTRING_SRC="$SOURCE_DIR/library/mystring.cpp"

BUILD_DIR="$SOURCE_DIR/build"
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

g++ -std=c++11 -c -o "$BUILD_DIR/main.o" "$MAIN_SRC"
g++ -std=c++11 -c -o "$BUILD_DIR/mystring.o" "$MYSTRING_SRC"

ar rcs "$BUILD_DIR/lib$LIB_NAME.a" "$BUILD_DIR/mystring.o"

# g++ -std=c++11 -o "$BUILD_DIR/main" "$BUILD_DIR/main.o" -L"$BUILD_DIR" -l"$LIB_NAME"
# Dynamic Library generation
g++ -std=c++11 -shared -fPIC -o "$BUILD_DIR/lib$LIB_NAME.so" "$BUILD_DIR/mystring.o"

# Static and Dynamic executables respectively
g++ -std=c++11 -o "$BUILD_DIR/main_static" "$BUILD_DIR/main.o" -L"$BUILD_DIR" -l"$LIB_NAME"
g++ -std=c++11 "$BUILD_DIR/main.o" -o "$BUILD_DIR/main_dynamic"  "$BUILD_DIR/lib$LIB_NAME.so"