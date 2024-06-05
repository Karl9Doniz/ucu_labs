cmake_minimum_required(VERSION 3.26)
project(lab_3_POC)

set(CMAKE_CXX_STANDARD 17)

add_executable(lab3-strings-usage-ahitoliev_ivanyshyn_pavliuk main.cpp)

add_subdirectory(Ahitoliev_Pavluk_Ivanyshun_string)

add_library(mystringlib_to_check SHARED Ahitoliev_Pavluk_Ivanyshun_string/mystring.cpp)
target_link_libraries(lab3-strings-usage-ahitoliev_ivanyshyn_pavliuk mystringlib_to_check)
