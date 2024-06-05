// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com
#include <iostream>
#include <iostream>
#include <string>
#include <exception>
#include <limits>
#include "mystring.hpp"
#include <cassert>


////insert int argc, char* argv[]) into main method signature
int main() {
    //command_line_options_t command_line_options{argc, argv};
    //std::cout << "A flag value: " << command_line_options.get_A_flag() << std::endl;

//    my_str_t str1(10, 'T'); // Regular constructor with given size
//    my_str_t str2("Hello, world!"); // Create a string from a C-string
//    my_str_t str3(std::string("C++ style string")); // Construct a string from a C++ string
//
//    my_str_t str4(str2); // Copy the str2 into str4
//
//    my_str_t str5(10, 'X');
//    str5 = str3; // Assign the value of str3 to str5
//    my_str_t str6 = "zzz";
//    my_str_t str7 = "aaa";
//    str6.swap(str7);// Using Swap function for str4 to str3
//
//    std::cout<< "str6: " << str6 << " str7: " << str7 << std::endl;
//    std::cout << str3[15] << std::endl;
//    const my_str_t& const_str = str3; //Const indexation object
//    std::cout << const_str[1] << std::endl;
//    std::cout << str3.at(0) << std::endl;
//    std::cout<<const_str.at(1)<< std::endl;
//
//    my_str_t str("Hello, World!");
//
//    // c_str() method
//    const char* cstr = str.c_str();
//    std::cout << "C-String Representation: " << cstr << std::endl;
//
//    // insert methods
//    str.insert(5, ','); // Insert a comma at index 5
//    std::cout << "After Inserting a Comma: " << str << std::endl;
//
//    my_str_t insertedStr("Wonderful ");
//    str.insert(7, insertedStr); // Insert "Wonderful " at index 7
//    std::cout << "After Inserting 'Wonderful ': " << str << std::endl;
//
//    const char* cstrToInsert = "This is a ";
//    str.insert(0, cstrToInsert); // Insert "This is a " at the beginning
//    std::cout << "After Inserting 'This is a ': " << str << std::endl;
//
//    // append methods
//    my_str_t appendedStr(" Day!");
//    str.append(appendedStr); // Append " Day!"
//    std::cout << "After Appending ' Day!': " << str << std::endl;
//    std::cout << 'f' << std::endl;
//    std::cout << str2.c_str() << std::endl;
//
//    const char* cstrToAppend = " How are you?";
//    str.append(cstrToAppend); // Append " How are you?"
//    std::cout << "After Appending ' How are you?': " << str << std::endl;
//
//    // erase method
//    str.erase(0, 12); // Erase the first 12 characters
//    std::cout << "After Erasing the First 12 Characters: " << str << std::endl;
//
//    // size and capacity
//    std::cout << "Size: " << str.size() << ", Capacity: " << str.capacity()<<std::endl;
//
//    // resize
//    str.resize(20, '*'); // Resize to a length of 20 with '*' padding
//    std::cout << "After Resizing to 20 Characters with '*': " << str << std::endl;
//
//    // shrink_to_fit method
//    str.shrink_to_fit();
//    size_t cap = str.capacity();
//    std::cout << "After Shrink to Fit, Capacity: " << cap << std::endl;
//
//    // reserve method
//    str.reserve(50); // Reserve space for 50 characters
//    size_t capa = str.capacity();
//    std::cout << "After Reserving Space for 50 Characters, Capacity: " << capa << std::endl;
//
//    // clear method
//    str.clear();
//    std::cout << "After Clearing: " << str << std::endl;
//
//    // const c_str() method
//    const my_str_t constStr("Const String");
//    const char* constCstr = constStr.c_str();
//    std::cout << "Const C-String Representation: " << constCstr << std::endl;
//
//    // find methods
//    str = "C++ strings are totally cool";
//    size_t foundIndex = str.find('o');
//    std::cout << "Index of 'o': " << foundIndex << std::endl;
//
//    foundIndex = str.find("are");
//    std::cout << "Index of 'are': " << foundIndex << std::endl;
//
//    foundIndex = str.find(std::string ("o"), 5); // Start searching from index 5
//    std::cout << "Index of 'o' starting from index 5: "<<std::endl;
//
//    // substr method
//    my_str_t substring = str.substr(6, 5); // Get a substring starting at index 6 with length 5
//    std::cout << "Substring: " << substring << std::endl;
//
//    my_str_t str_1 = "Hello";
//    my_str_t str_2 = "World";
//    my_str_t str_3 = "Hello";
//    const char* cst = "Hello";
//
//    // Demonstrate comparison operators
//    std::cout << "str_1 == str_2: " << (str_1 == str_2) << std::endl; // false
//    std::cout << "str_1 == str_3: " << (str_1 == str_3) << std::endl; // true
//    std::cout << "str_1 != str_2: " << (str_1 != str_2) << std::endl; // true
//    std::cout << "str_1 != str_3: " << (str1 != str3) << std::endl; // false
//    std::cout << "str_1 < str_2: " << (str_1 < str_2) << std::endl;   // true
//    std::cout << "str_1 <= str_3: " << (str_1 <= str_3) << std::endl; // true
//    std::cout << "str_2 > str_3: " << (str_2 > str_3) << std::endl;   // true
//    std::cout << "str_2 >= str_1: " << (str_2 >= str_1) << std::endl; // true
//    std::cout << "cst == str_1: " << (cst == str_1) << std::endl; // true
//
//
//    my_str_t str_a = "apple";
//    my_str_t str_b = "banana";
//    my_str_t str_c(str_a);
//    const char* c_str = "cherry";
//
//    // True, so output is 1
//    std::cout << (str_a <= str_b) << std::endl;
//    std::cout << (str_a < cstr) << std::endl;
//    // False, hence output is 0
//    std::cout << (c_str <= str_b) << std::endl;
//
//    // Also testing equality
//    std::cout << (str_c <= str_a) << std::endl;

    my_str_t test_erase("Hi, hello, hola, ciao, buongiorno!!!");
    std::cout << "Size of test_erase: " << test_erase.size() << std::endl;
    std::cout << "Capacity of test_erase: " << test_erase.capacity() << std::endl;
    std::cout << "Is the size of test_erase equal to 36? " << (test_erase.size() == 36 ? "Yes" : "No") << std::endl;
    std::cout << "Is the capacity of test_erase greater than or equal to its size? " << (test_erase.capacity() >= test_erase.size() ? "Yes" : "No") << std::endl;

    // Erase inside
    test_erase.erase(3, 7);
    std::cout << "After erasing from position 3, length 7, test_erase is: " << test_erase << std::endl;
    std::cout << "Is the result equal to \"Hi, hola, ciao, buongiorno!!!\"? " << (test_erase == "Hi, hola, ciao, buongiorno!!!" ? "Yes" : "No") << std::endl;
    std::cout << "Is the size of test_erase equal to 29? " << (test_erase.size() == 29 ? "Yes" : "No") << std::endl;
    std::cout << "Is the capacity of test_erase greater than or equal to its size? " << (test_erase.capacity() >= test_erase.size() ? "Yes" : "No") << std::endl;

    // Erase from the start
    test_erase.erase(0, 4);
    std::cout << "After erasing from position 0, length 4, test_erase is: " << test_erase << std::endl;
    std::cout << "Is the result equal to \"hola, ciao, buongiorno!!!\"? " << (test_erase == "hola, ciao, buongiorno!!!" ? "Yes" : "No") << std::endl;
    std::cout << "Is the size of test_erase equal to 25? " << (test_erase.size() == 25 ? "Yes" : "No") << std::endl;
    std::cout << "Is the capacity of test_erase greater than or equal to its size? " << (test_erase.capacity() >= test_erase.size() ? "Yes" : "No") << std::endl;

    // Erase to the end (out of bounds)
    try {
        test_erase.erase(22, 10);
        std::cout << "Erase with position 22 and length 10 did not throw an exception." << std::endl;
    } catch (const std::out_of_range& e) {
        std::cout << "Exception caught: " << e.what() << std::endl;
    }
    std::cout << "After attempting out-of-bounds erase, test_erase is: " << test_erase << std::endl;
    std::cout << "Is the result equal to \"hola, ciao, buongiorno\"? " << (test_erase == "hola, ciao, buongiorno" ? "Yes" : "No") << std::endl;
    std::cout << "Is the size of test_erase equal to 22? " << (test_erase.size() == 22 ? "Yes" : "No") << std::endl;
    std::cout << "Is the capacity of test_erase greater than or equal to its size? " << (test_erase.capacity() >= test_erase.size() ? "Yes" : "No") << std::endl;

    // Erase from the end
    test_erase.erase(test_erase.size()-1, 10);
    std::cout << "After erasing from the end, test_erase is: " << test_erase << std::endl;
    std::cout << "Is the result equal to \"hola, ciao, buongiorn\"? " << (test_erase == "hola, ciao, buongiorn" ? "Yes" : "No") << std::endl;
    std::cout << "Is the size of test_erase equal to 21? " << (test_erase.size() == 21 ? "Yes" : "No") << std::endl;
    std::cout << "Is the capacity of test_erase greater than or equal to its size? " << (test_erase.capacity() >= test_erase.size() ? "Yes" : "No") << std::endl;

    // Bad range
    try {
        test_erase.erase(test_erase.size()+5, 10);
        std::cout << "Erase with bad range did not throw an exception." << std::endl;
    } catch (const std::out_of_range& e) {
        std::cout << "Exception caught: " << e.what() << std::endl;
    }
    std::cout << "After attempting erase with bad range, test_erase is: " << test_erase << std::endl;
    std::cout << "Is the result equal to \"hola, ciao, buongiorn\"? " << (test_erase == "hola, ciao, buongiorn" ? "Yes" : "No") << std::endl;
    std::cout << "Is the size of test_erase equal to 21? " << (test_erase.size() == 21 ? "Yes" : "No") << std::endl;
    std::cout << "Is the capacity of test_erase greater than or equal to its size? " << (test_erase.capacity() >= test_erase.size() ? "Yes" : "No") << std::endl;

    // Size is zero
    test_erase.erase(3, 0);
    std::cout << "After erasing with zero length, test_erase is: " << test_erase << std::endl;
    std::cout << "Is the result equal to \"hola, ciao, buongiorn\"? " << (test_erase == "hola, ciao, buongiorn" ? "Yes" : "No") << std::endl;
    std::cout << "Is the size of test_erase equal to 21? " << (test_erase.size() == 21 ? "Yes" : "No") << std::endl;
    std::cout << "Is the capacity of test_erase greater than or equal to its size? " << (test_erase.capacity() >= test_erase.size() ? "Yes" : "No") << std::endl;

    // Erase everything
    test_erase.erase(0, 30);
    std::cout << "After erasing everything, test_erase is: " << test_erase << std::endl;
    std::cout << "Is the result equal to \"\"? " << (test_erase == "" ? "Yes" : "No") << std::endl;
    std::cout << "Is the size of test_erase equal to 0? " << (test_erase.size() == 0 ? "Yes" : "No") << std::endl;
    std::cout << "Is the capacity of test_erase greater than or equal to its size? " << (test_erase.capacity() >= test_erase.size() ? "Yes" : "No") << std::endl;

    // Erase empty string
    test_erase.erase(0, 20);
    std::cout << "After attempting to erase an empty string, test_erase is: " << test_erase << std::endl;
    std::cout << "Is the result equal to \"\"? " << (test_erase == "" ? "Yes" : "No") << std::endl;
    std::cout << "Is the size of test_erase equal to 0? " << (test_erase.size() == 0 ? "Yes" : "No") << std::endl;
    std::cout << "Is the capacity of test_erase greater than or equal to its size? " << (test_erase.capacity() >= test_erase.size() ? "Yes" : "No") << std::endl;
    return 0;
}


