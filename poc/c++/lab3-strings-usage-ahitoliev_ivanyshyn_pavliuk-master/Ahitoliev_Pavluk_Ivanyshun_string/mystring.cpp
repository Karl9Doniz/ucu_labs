// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com
#include <iostream>
#include "mystring.hpp"

// 
my_str_t::my_str_t (size_t size, char initial) : size_m{size} {
    capacity_m = size_m - (size_m%16) + 16;
    data_m = new char[ capacity_m + 1];
    memset(data_m, initial, size_m);
    data_m[size_m] = '\0';
}

// 
my_str_t::my_str_t(const char* cstr) : size_m {strlen(cstr)} {
    capacity_m = size_m - (size_m%16) + 16;
    data_m = new char[capacity_m + 1];
    memcpy(data_m, cstr, size_m+1);
    data_m[size_m] = '\0';
}


// 
my_str_t::my_str_t(const std::string &str) : size_m {str.size()}  {
    capacity_m = size_m - (size_m%16) + 16;
    data_m = new char[capacity_m+1];
    memcpy(data_m, str.data(), size_m);
    data_m[size_m] = '\0';
}


// 
my_str_t::my_str_t(const my_str_t &mystr) :
    capacity_m {mystr.capacity_m}, size_m {mystr.size_m} {
    data_m = new char[capacity_m+1];
    memcpy(data_m, mystr.data_m, size_m);
    data_m[size_m] = '\0';
}


// 
my_str_t &my_str_t::operator=(const my_str_t &mystr) {
    if (this == &mystr){
        return *this;
    }
    delete[] data_m;
    capacity_m = mystr.capacity_m;
    size_m = mystr.size_m;
    data_m = new char[capacity_m+1];
    memcpy(data_m, mystr.data_m, size_m+1);
    return *this;
}


// 
void my_str_t::swap(my_str_t &other) noexcept {
    std::swap(data_m, other.data_m);
    std::swap(size_m, other.size_m);
    std::swap(capacity_m, other.capacity_m);
}


// 
char &my_str_t::operator[](size_t idx) {
    return data_m[idx];
}


// 
const char &my_str_t::operator[](size_t idx) const {
    return data_m[idx];
}


// 
char &my_str_t::at(size_t idx) {
    if (idx >= size_m){
        throw std::out_of_range("Index out of range");
    }
    return data_m[idx];
}


// 
const char &my_str_t::at(size_t idx) const {
    if (idx >= size_m){
        throw std::out_of_range("Index out of range");
    }
    return data_m[idx];
}


// 
void my_str_t::reserve(size_t new_capacity) {
    if (new_capacity > capacity_m){
        capacity_m = new_capacity;
        char *miwa = new char[new_capacity+1];
        memcpy(miwa, data_m, size_m+1);
        delete[] data_m;
        data_m = miwa;
    }
}

// 
void my_str_t::shrink_to_fit() {
    capacity_m = size_m;
    char *miwa = new char[size_m+1];
    memcpy(miwa, data_m, size_m+1);
    delete[] data_m;
    data_m = miwa;
}

//
void my_str_t::resize(size_t new_size, char new_char){
    if (new_size < size_m){
        size_m = new_size;
    }
    else if (new_size > size_m && new_size <= capacity_m){
        char* new_data = new char[capacity_m + 1];
        memcpy(new_data, data_m, size_m);
        memset(new_data+size_m, new_char, new_size - size_m);
        delete[] data_m;
        data_m = new_data;
        size_m = new_size;
        data_m[size_m] = '\0';
    }
    else if (new_size > capacity_m){
        reserve(new_size);
        char* new_data = new char[capacity_m + 1];
        memcpy(new_data, data_m, size_m);
        memset(new_data+size_m, new_char, new_size - size_m);
        delete[] data_m;
        data_m = new_data;
        size_m = new_size;
        data_m[size_m] = '\0';
    }
}

// 
void my_str_t::clear() {
    size_m = 0;
}

// 
void my_str_t::insert(size_t idx, const my_str_t &str) {
    if( idx > size_m) {
        throw std::out_of_range("Index out of range");
    }
    size_m += str.size_m;
    size_t new_cap = size_m - (size_m%16) + 16;
    capacity_m = (new_cap > capacity_m) ? new_cap : capacity_m;
    memmove(data_m+str.size_m+idx, data_m+idx, size_m-idx-str.size_m+1);
    memcpy(data_m+idx, str.data_m, str.size_m);

}

// 
void my_str_t::insert(size_t idx, char c) {
    if( idx > size_m) {
        throw std::out_of_range("Index out of range");
    }
    ++size_m;
    size_t new_cap = size_m - (size_m%16) + 16;
    capacity_m = (new_cap > capacity_m) ? new_cap : capacity_m;
    memmove(data_m+idx+1, data_m+idx, size_m-idx);
    data_m[idx] = c;
}

// 
void my_str_t::insert(size_t idx, const char *cstr) {
    if( idx > size_m) {
        throw std::out_of_range("Index out of range");
    }
    size_t size = strlen(cstr);
    size_m += size;
    size_t new_cap = size_m - (size_m%16) + 16;
    capacity_m = (new_cap > capacity_m) ? new_cap : capacity_m;
    memmove(data_m+size+idx, data_m+idx, size_m-idx-size+1);
    memcpy(data_m+idx, cstr, size);
}

// 
void my_str_t::append(const my_str_t &str) {
    size_m+=str.size_m;
    size_t new_cap = size_m - (size_m%16) + 16;
    capacity_m = (new_cap > capacity_m) ? new_cap : capacity_m;
    memmove(data_m+size_m-str.size_m, str.data_m, str.size_m+1);
}

// 
void my_str_t::append(char c) {
    ++size_m;
    size_t new_cap = size_m - (size_m%16) + 16;
    capacity_m = (new_cap > capacity_m) ? new_cap : capacity_m;
    memmove(data_m+size_m-1, &c, 1);
}

// 
void my_str_t::append(const char *cstr) {
    size_t size = strlen(cstr);
    size_m+=size;
    size_t new_cap = size_m - (size_m%16) + 16;
    capacity_m = (new_cap > capacity_m) ? new_cap : capacity_m;
    memmove(data_m+size_m-size, cstr, size);
}

// 
size_t my_str_t::size() const noexcept {
    return size_m;
}
// 
size_t my_str_t::capacity() const noexcept {
    return capacity_m;
}

// 
void my_str_t::erase(size_t begin, size_t size) {
    if(begin > size_m){
        throw std::out_of_range("Index of begin out of range");
    }
    else if(size > size_m-begin){
        char *miwa = new char[capacity_m+1];
        memcpy(miwa,data_m,begin);
        delete[] data_m;
        data_m = miwa;
        size_m = begin;
        capacity_m = begin-(begin%16)+16;
    }
    else{
        char *miwa = new char[capacity_m+1];
        memmove(miwa,data_m,begin);
        memmove(miwa+begin,data_m+begin+size,size_m-begin-size);
        delete[] data_m;
        data_m = miwa;
        size_m = size_m-size;
        capacity_m = size_m-(size_m%16)+16;
    }
}

// 
size_t my_str_t::find(char c, size_t idx) {
    if (size_m == 0){
        return my_str_t::not_found;
    }
    if (idx >= size_m) {
        throw std::out_of_range("Index out of range");
    }
    for (size_t i = idx; i < size_m; ++i){
        if (data_m[i] == c){
            return i;
        }
    }
    return my_str_t::not_found;
}

// 
size_t my_str_t::find(const std::string &str, size_t idx) {
    if (size_m == 0){
        return my_str_t::not_found;
    }
    if (idx > size_m) {
        throw std::out_of_range("Index out of range");
    }
    size_t len = str.length();
    for (size_t i = idx; i < size_m - len + 1; ++i) {
        if (data_m[i] == str[0]){
            for (size_t j = 0; j < len; ++j){
                if (data_m[i+j] == str[j] && j == len-1){
                    return i;
                }
            }
        }
    }
    return my_str_t::not_found;
}

// 
size_t my_str_t::find(const char *cstr, size_t idx) {
    if (size_m == 0){
        return my_str_t::not_found;
    }
    if (idx > size_m) {
        throw std::out_of_range("Index out of range");
    }
    size_t len = strlen(cstr);
    for (size_t i = idx; i < size_m - len + 1; ++i) {
        if (data_m[i] == cstr[0]){
            for (size_t j = 0; j < len; ++j){
                if (data_m[i+j] == cstr[j] && j == len-1){
                    return i;
                }
            }
        }
    }
    return my_str_t::not_found;
}

// 
my_str_t my_str_t::substr(size_t begin, size_t size) {
    if (begin > size_m) {
        throw std::out_of_range("Index out of range");
    }
    size_t len = (begin + size < size_m) ? size: size_m - begin;
    char* miwa = new char [len+1];
    memcpy(miwa, data_m + begin, len);
    miwa[len] = '\0';
    my_str_t miwik = my_str_t(miwa);
    delete[] miwa;
    return miwik;
}

// 
my_str_t::~my_str_t() {
    delete[] data_m;
}

//  all bool operators
bool operator==(const my_str_t &str1, const my_str_t &str2){
    size_t len1 = str1.size();
    size_t len2 = str2.size();
    if (len1 != len2){
        return false;
    }
    for (size_t i = 0; i < len1; ++i){
        if (str1[i] != str2[i]){
            return false;
        }
    }
    return true;
}

bool operator!=(const my_str_t &str1, const my_str_t &str2){
    return !(str1 == str2);
}


bool operator<(const my_str_t &str1, const my_str_t &str2){
    size_t len1 = str1.size();
    size_t len2 = str2.size();
    size_t siz = (len1 >= len2) ? len2 : len1;
    for (size_t i = 0; i < siz; ++i){
        if (str1[i] > str2[i]){
            return false;
        }
        else if (str1[i] < str2[i]){
            return true;
        }
    }
    return len1 < len2;
}

bool operator>(const my_str_t &str1, const my_str_t &str2){
    return !(str1<str2) && str2!=str1;
}

bool operator<=(const my_str_t &str1, const my_str_t &str2){
    return  str1 < str2 || str1 == str2;
}

bool operator>=(const my_str_t &str1, const my_str_t &str2){
    return  str2 < str1 || str1 == str2;
}

bool operator==(const my_str_t& str1 , const char* cstr2 ){
    size_t len1 = str1.size();
    size_t len2 = strlen(cstr2);
    if (len1 != len2){
        return false;
    }
    for (size_t i = 0; i < len1; ++i){
        if (str1[i] != cstr2[i]){
            return false;
        }
    }
    return true;
}

bool operator!=(const my_str_t &str1, const char* cstr2){
    return !(str1 == cstr2);
}

bool operator<(const my_str_t& str1 , const char* cstr2){
    size_t len1 = str1.size();
    size_t len2 = strlen(cstr2);
    size_t siz = (len1 >= len2) ? len2 : len1;
    for (size_t i = 0; i < siz; ++i){
        if (str1[i] > cstr2[i]){
            return false;
        }
        else if (str1[i] < cstr2[i]){
            return true;
        }
    }
    return len1 < len2;
}

bool operator>(const my_str_t &str1, const char* cstr2){
    return !(str1 < cstr2) && (str1 != cstr2);
}

bool operator<=(const my_str_t &str1, const char* cstr2){
    return str1 < cstr2 || str1 == cstr2;
}

bool operator>=(const my_str_t &str1, const char* cstr2){
    return !(str1 < cstr2) || str1 == cstr2;
}

bool operator==(const char* cstr1 , const my_str_t& str2 ){
    size_t len1 = strlen(cstr1);
    size_t len2 = str2.size();
    if (len1 != len2){
        return false;
    }
    for (size_t i = 0; i < len1; ++i){
        if (cstr1[i] != str2[i]){
            return false;
        }
    }
    return true;
}

bool operator!=(const char* cstr1, const my_str_t &str2){
    return !(cstr1 == str2);
}

bool operator<( const char* cstr1 , const my_str_t& str2 ){
    size_t len1 = strlen(cstr1);
    size_t len2 = str2.size();
    size_t siz = (len1 >= len2) ? len2 : len1;
    for (size_t i = 0; i <= siz; ++i){
        if (cstr1[i] > str2[i]){
            return false;
        }
        else if (cstr1[i] < str2[i]){
            return true;
        }
    }
    return len1 < len2;
}

bool operator>( const char* cstr1 , const my_str_t& str2 ){
    return !(cstr1 < str2) && (cstr1 != str2);
}

bool operator<=( const char* cstr1 , const my_str_t& str2 ){
    return cstr1 < str2 || cstr1 == str2;
}

bool operator>=( const char* cstr1 , const my_str_t& str2 ){
    return !(cstr1 < str2) || cstr1 == str2;
}

std::ostream& operator<<(std::ostream& stream, const my_str_t& str){
    stream << str.c_str();
    return stream;
}

std::istream& operator>>(std::istream& stream, my_str_t& str) {
    str.clear();
    char ch;
    while(stream.get(ch)&&isspace(ch)){
    }
    stream.unget();
    while (stream.get(ch) && !isspace(ch) && ch != '\n') {
        str.append(ch);
    }
    if(stream.eof()) {
        stream.unget();
    }
    str.shrink_to_fit();
    return stream;
}

std::istream &readline(std::istream &stream, my_str_t &str) {
        str.clear();
    char ch;
    while (stream.get(ch) && ch!='\n') {
        str.append(ch);
    }

    return stream;
}

