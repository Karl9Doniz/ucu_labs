// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com
#include <iostream>
#include <cstring>


#include "mystring.hpp"

//Bohdan
void my_str_t::reserve(size_t new_capacity) {
    if (capacity_m > new_capacity) {
        return;
    }
    char *new_data = new char[new_capacity + 1];
    for (size_t i = 0; i < size_m; i++) {
        new_data[i] = data_m[i];
    }
    delete[] data_m;
    this->capacity_m = new_capacity;
    this->data_m = new_data;
}
//Bohdan
void my_str_t::shrink_to_fit() {
    if (capacity_m == size_m + 1){
        return;
    }
    capacity_m = size_m + 1;
    char* data_n = new char[capacity_m];
    std::memcpy(data_n, data_m, capacity_m);
    delete[] data_m;
    data_m = data_n;
};
//Bohdan
void my_str_t::resize(size_t new_size, char new_char){
    if (new_size < size_m){
        size_m = new_size;
        data_m[size_m]  = '\0';
    }
    else if(new_size > size_m && new_size < capacity_m){
        for (size_t i = size_m; i < new_size; i++){
            data_m[i] = new_char;
        }
        size_m = new_size;
    }
    else{
        reserve(new_size);
        for (size_t i = size_m; i < new_size; i++){
            data_m[i] = new_char;
        }
        size_m = new_size;
    }
}
//Bohdan
void my_str_t::clear() {
    size_m = 0;
    data_m[size_m] = '\0';
}

//
void my_str_t::insert(size_t idx, const my_str_t& str) {
    insert(idx, str.data_m);
}

void my_str_t::insert(size_t idx, char c) {
    if (idx > size_m) {
        throw std::out_of_range("Index out of range");
    }

    if (size_m + 1 >= capacity_m) {
        reserve(capacity_m * 2 + 1);
    }

    std::memmove(data_m + idx + 1, data_m + idx, size_m - idx);

    data_m[idx] = c;
    data_m[++size_m] = '\0';
}

void my_str_t::insert(size_t idx, const char* cstr) {
    if (idx > size_m) {
        throw std::out_of_range("Index out of range");
    }

    size_t cstr_size = std::strlen(cstr);

    if (size_m + cstr_size >= capacity_m) {
        reserve(capacity_m * 2 + cstr_size + 1);
    }

    std::memmove(data_m + idx + cstr_size, data_m + idx, size_m - idx);
    std::memcpy(data_m + idx, cstr, cstr_size);

    size_m += cstr_size;
    data_m[size_m] = '\0';
}

//Bohdan
void my_str_t::append(const my_str_t& str){
    size_t other_size = str.size_m;
    char *data_other = str.data_m;
    if (other_size + size_m > capacity_m){
        reserve(other_size + size_m);
    }

    for (size_t i = 0; i < other_size; i++){
        data_m[size_m + i] = data_other[i];
    }
    size_m += other_size;
}
//Bohdan
void my_str_t::append(char c){
    if (1 + size_m > capacity_m){
        reserve(1 + size_m);
    }

    data_m[size_m] = c;
    size_m += 1;
}
//Bohdan
void my_str_t::append(const char* cstr){
    size_t cstr_size = std::strlen(cstr);
    if (size_m + cstr_size > capacity_m){
        reserve(size_m + cstr_size);
    }
    for (size_t i = 0; i < cstr_size; i++){
        data_m[size_m + i] = cstr[i];
    }
    size_m += cstr_size;
}
// Ihor
void my_str_t::erase(size_t begin, size_t size) {
    if (begin > size_m) {
        throw std::out_of_range("Index out of range");
    }
    size_t erase = size_m - begin;
    if (size < erase) {
        erase = size;
    }
    for (size_t i = begin; i + erase < size_m; i++) {
        data_m[i] = data_m[i + erase];
    }
    size_m -= erase;
    data_m[size_m] = '\0';
}
// Ihor
size_t my_str_t::size() const noexcept {
    return size_m;
}
// Ihor
size_t my_str_t::capacity() const{
    return capacity_m;
}
// Ihor
const char* my_str_t::c_str() const {
    if (size_m > 0) {
        data_m[size_m] = '\0';
        return data_m;
    }
    return "";
}
// Ihor
size_t my_str_t::find(char c, size_t idx /* = 0 */) {
    if (idx == 0){
        return -1;
    }
    if (idx >= size_m) {
        throw std::out_of_range("Index out of range");
    }
    for (size_t i = idx; i < size_m; ++i) {
        if (data_m[i] == c) {
            return i;
        }
    }
    return -1;
}
// Ihor
size_t my_str_t::find(const std::string& str, size_t idx /* = 0 */) {
    size_t str_length = str.length();
    if (size_m == 0) {
        return -1;
    }
    if (str_length == 0) {
        return -1;
    }if (idx > size_m) {
        throw std::out_of_range("Index out of range");
    }

    while (idx < size_m - str_length) {
        size_t i = 0;
        while (i < str_length && std::tolower(data_m[idx + i]) == std::tolower(str[i])) {
            ++i;
        }

        if (i == str_length) {
            return idx;
        }

        ++idx;
    }

    return -1;
}
// Ihor
size_t my_str_t::find(const char* cstr, size_t idx /* = 0 */) {
    size_t cstr_length = strlen(cstr);
    if (size_m == 0) {
        return -1;
    }
    if (cstr_length == 0) {
        return -1;
    }
    if (idx > size_m) {
        throw std::out_of_range("Index out of range");
    }
    while (idx < size_m - cstr_length) {
        size_t i = 0;
        while (i < cstr_length && std::tolower(data_m[idx + i]) == std::tolower(cstr[i])) {
            ++i;
        }
        if (i == cstr_length) {
            return idx;
        }
        ++idx;
    }
    return -1;
}

// Ihor
my_str_t my_str_t::substr(size_t begin, size_t size) {
    if (begin > size_m) {
        throw std::out_of_range("Index out of range");
    }
    if (begin + size > size_m){
        size = size_m - begin;
    }
    my_str_t result(size, ' ');
    for (size_t i = 0; i < size; ++i) {
        result.data_m[i] = data_m[begin + i];
    }
    return result;
}

//Andriy
my_str_t::my_str_t(size_t size, char initial):capacity_m(size), size_m(size) {
    data_m = new char[capacity_m + 1];

    for (size_t i = 0; i < size; ++i) {
        data_m[i] = initial;
    }

    if (size > 0) {
        data_m[size] = '\0';
    }
}
//Andriy
my_str_t::my_str_t(const char* c_str) {
    size_m = strlen(c_str);
    capacity_m = size_m + 1;
    data_m = new char[capacity_m];

    for (size_t i = 0; i < size_m; ++i) {
        data_m[i] = c_str[i];
    }

    data_m[size_m] = '\0';
}
//Andriy
my_str_t::my_str_t(const std::string& str) {
    size_m = str.length();
    capacity_m = size_m + 1;
    data_m = new char[capacity_m];

    for (size_t i = 0; i < size_m; ++i) {
        data_m[i] = str[i];
    }

    data_m[size_m] = '\0';
}
//Andriy
my_str_t::my_str_t(const my_str_t& my_str) : capacity_m(my_str.capacity_m), size_m(my_str.size_m) {
    data_m = new char[capacity_m + 1];

    // Copy each character from my_str.data_m to data_m
    for (size_t i = 0; i < size_m; ++i) {
        data_m[i] = my_str.data_m[i];
    }

    // Add the null-terminator to the end
    data_m[size_m] = '\0';
}
//Andriy
my_str_t& my_str_t::operator=(const my_str_t& my_str) {
    if (this != &my_str) {
        my_str_t temp(my_str);
        this->swap(temp);
    }
    return *this;
}

// Swap function
//Andriy
void my_str_t::swap(my_str_t& other) noexcept {
    char* temp_data = this->data_m;
    size_t temp_capacity = this->capacity_m;
    size_t temp_size = this->size_m;

    this->data_m = other.data_m;
    this->capacity_m = other.capacity_m;
    this->size_m = other.size_m;

    other.data_m = temp_data;
    other.capacity_m = temp_capacity;
    other.size_m = temp_size;
}

// [] operator (const version)
//Andriy
const char& my_str_t::operator[](size_t idx) const {
    return this->data_m[idx];
}

// [] operator (non-const version)
//Andriy
char& my_str_t::operator[](size_t idx) {
    return this->data_m[idx];
}
//Andriy
const char& my_str_t::at(size_t idx) const {
    if (idx > size_m) {
        throw std::out_of_range("Index out of range");
    }
    return this->data_m[idx];
}

// at member function (non-const version) with bounds checking
//Andriy
char& my_str_t::at(size_t idx) {
    if (idx >= size_m) {
        throw std::out_of_range("Index out of bounds");
    }
    return this->data_m[idx];
}
//Andriy
my_str_t::~my_str_t() {
    delete[] data_m;
    size_m = 0;
    capacity_m = 0;
}
//Bohdan
bool operator==(const my_str_t& str1, const my_str_t& str2){
    bool flag = true;

    if (str1.size() == str2.size()){
        for (size_t i = 0; i < str1.size() ; i++) {
            if (str1[i] != str2[i]){
                flag = false;
                break;
            }
        }
    }
    else
        flag = false;
    return flag;
}
//Bohdan
bool operator!=(const my_str_t& str1, const my_str_t& str2){
    if (str1 == str2){
        return false;
    }
    return true;
}
//Bohdan
bool operator==(const my_str_t& str1, const char* cstr2){
    size_t size_str1 = str1.size();
    if (size_str1 != std::strlen(cstr2)){
        return false;
    }
    const char* data_str = str1.c_str();
    for (size_t i = 0; i < size_str1; i++){
        if(data_str[i] != cstr2[i]){
            return false;
        }
    }
    return true;
}
//Bohdan
bool operator!=(const my_str_t& str1, const char* cstr2){
    if (str1 == cstr2){
        return false;
    }
    return true;
}
//Bohdan
bool operator==(const char* cstr1, const my_str_t& str2){
    if (str2 == cstr1){
        return true;
    }
    return false;
}
//Bohdan
bool operator!=(const char* cstr1, const my_str_t& str2){
    if (str2 == cstr1){
        return false;
    }
    return true;
}
//Andriy
bool operator<(const my_str_t& str1, const my_str_t& str2) {
    const char* data1 = str1.c_str();
    const char* data2 = str2.c_str();
    size_t size1 = str1.size();
    size_t size2 = str2.size();

    for (size_t i = 0; i < size1 && i < size2; ++i) {
        if (data1[i] < data2[i]) {
            return true;
        } else if (data1[i] > data2[i]) {
            return false;
        }
    }

    return size1 < size2;
}
//Andriy
bool operator<(const my_str_t& str1, const char* cstr2) {
    const char* data1 = str1.c_str();
    size_t size1 = str1.size();
    size_t size2 = std::strlen(cstr2);

    for (size_t i = 0; i < size1 && i < size2; ++i) {
        if (data1[i] < cstr2[i]) {
            return true;
        } else if (data1[i] > cstr2[i]) {
            return false;
        }
    }

    return size1 < size2;
}
//Andriy
bool operator<(const char* cstr1, const my_str_t& str2) {
    const char* data2 = str2.c_str();
    size_t size1 = std::strlen(cstr1);
    size_t size2 = str2.size();

    for (size_t i = 0; i < size1 && i < size2; ++i) {
        if (cstr1[i] < data2[i]) {
            return true;
        } else if (cstr1[i] > data2[i]) {
            return false;
        }
    }

    return size1 < size2;
}
//Andriy
bool operator<=(const my_str_t& str1, const my_str_t& str2) {
    return !(str2 < str1);
}
//Andriy
bool operator<=(const my_str_t& str1, const char* cstr2) {
    return !(cstr2 < str1);
}
//Andriy
bool operator<=(const char* cstr1, const my_str_t& str2) {
    return !(str2 < cstr1);
}
// Ihor
bool operator> (const my_str_t& str1, const my_str_t& str2){
    const char* data1 = str1.c_str();
    const char* data2 = str2.c_str();
    size_t size1 = str1.size();
    size_t size2 = str2.size();

    for (size_t i = 0; i < size1 && i < size2; ++i) {
        if (data1[i] > data2[i]) {
            return true;
        } else if (data1[i] < data2[i]) {
            return false;
        }
    }

    return size1 > size2;
};
// Ihor
bool operator>=(const my_str_t& str1, const my_str_t& str2){
    return !(str1 > str2);
};
// Ihor
bool operator> (const my_str_t& str1, const char* cstr2){
    const char* data1 = str1.c_str();
    size_t size1 = str1.size();
    size_t size2 = std::strlen(cstr2);

    for (size_t i = 0; i < size1 && i < size2; ++i) {
        if (data1[i] > cstr2[i]) {
            return true;
        } else if (data1[i] < cstr2[i]) {
            return false;
        }
    }

    return size1 > size2;
};
// Ihor
bool operator>=(const my_str_t& str1, const char* cstr2){
    return !(str1 > cstr2);
};
// Ihor
bool operator> (const char* cstr1, const my_str_t& str2){
    const char* data2 = str2.c_str();
    size_t size1 = std::strlen(cstr1);
    size_t size2 = str2.size();

    for (size_t i = 0; i < size1 && i < size2; ++i) {
        if (cstr1[i] > data2[i]) {
            return true;
        } else if (cstr1[i] < data2[i]) {
            return false;
        }
    }

    return size1 > size2;
};
// Ihor
bool operator>=(const char* cstr1, const my_str_t& str2){
    return !(cstr1 > str2);
};
//Bohdan
std::ostream& operator<<(std::ostream& stream, const my_str_t& str) {
    stream << str.c_str();
    return stream;
}
//Bohdan
std::istream& operator>>(std::istream& stream, my_str_t& str) {
    str.clear();
    char c;
    while (stream.get(c) && std::isspace(c)) {
    }

    while (stream && !std::isspace(c)) {
        str.append(c);
        stream.get(c);
    }
    return stream;
}
//Bohdan
std::istream& readline(std::istream& stream, my_str_t& str){
    str.clear();
    char c;
    while (stream.get(c) && !(c == '\n')){
        str.append((c));
    }
    return stream;
}
