// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com
#include "mystring.hpp"
#include <iostream>
#include <fstream>

my_str_t testCin(const std::string& pathIn);
my_str_t testReadline(std::string path);
void printStr(const my_str_t& str);

int main() {
    //  tests
    std::cout << "\nTesting constructors\n";
    my_str_t test1_1(3,'A'); // constructor 1 (from chars)
    printStr(test1_1);

    char ClikeStr[] = {'C', '-', 'l', 'i', 'k', 'e', ' ', 'S', 't', 'r', 'i', 'n', 'g', '\0'};
    my_str_t test1_2(ClikeStr); // constructor 2(from C-like)
    printStr(test1_2);

    std::string CppStr = "(C++ String)";
    my_str_t test1_3(CppStr); // constructor 3(from C++ string)
    printStr(test1_3);

    std::cout << "\nTesting copy and '='\n";

    my_str_t test2_1 = test1_1; // Copy from our own class
    printStr(test2_1);

    test2_1 = test1_2; // "=" operator
    printStr(test2_1);

    std::cout << "\nTesting swap\n";
    my_str_t swpStr1("String No1");
    my_str_t swpStr2("String No2");

    swpStr1.swap(swpStr2); // testing swap

    std::cout << "Expecting 'String No2'\n";
    printStr(swpStr1); // must be "String No2"
    std::cout << "Expecting 'String No1'\n";
    printStr(swpStr2); // must be "String No1"
    //  tests

    //  tests
    my_str_t miwa4("Hello");
    std::cout<< "Testing 'operator[]': "<<"\n";
    std::cout<< "Given string: "<<miwa4<<"\n";
    std::cout<<"Expected output at index 1 - 'e'\n";
    std::cout<<"Output - "<<miwa4[1]<<'\n';


    const char* cstr1 = "C string";
    my_str_t miwa5(cstr1);
    std::cout<<'\n'<< "Testing 'operator[]' for C string: "<<"\n";
    std::cout<< "Given string: "<<miwa5<<"\n";
    std::cout<<"Expected output at index 2 - 's'\n";
    std::cout<<"Output - "<<miwa5[2]<<'\n';


    my_str_t miwa6("Hello");
    std::cout<<'\n'<< "Testing method 'at': "<<"\n";
    std::cout<< "Given string: "<<miwa6<<"\n";
    std::cout<<"Expected output at index 1 - 'e'\n";
    std::cout<<"Output - "<<miwa6.at(1)<<'\n';


    const char* cstr2 = "C string";
    const my_str_t miwa7(cstr2);
    std::cout<<'\n'<< "Testing method 'at': "<<"\n";
    std::cout<< "Given string: "<<miwa7<<"\n";
    std::cout<<"Expected output at index 2 - 's'\n";
    std::cout<<"Output - "<<miwa7.at(2)<<'\n';


    my_str_t miva8("Hello");
    std::cout<<"\n"<<"Testing 'reserve' method"<<'\n';
    std::cout<< "Given string: "<<miva8<<'\n';
    std::cout<< "Current capacity: "<< miva8.capacity()<<'\n';
    miva8.reserve(20);
    std::cout<<"After .reserve(20) expected output: "<<miva8.capacity()<<"(because it divides by 16 and greater than 20)"<<'\n';



    std::cout<<"\nTesting .shrink_to_fit method.\n";
    miva8.shrink_to_fit();
    std::cout<<"New capacity:"<<miva8.capacity()<<"\n";
    std::cout<<"\nTesting .resize method"<<"\n";
    miva8.resize(20,'r');
    std::cout<<"Output after Hello.resize(20,'r): \n";
    std::cout<<miva8<<'\n';
    //  tests


    //  tests
    std::cout << "\nTesting inserts\n";

    my_str_t test5("____");
    my_str_t inserter("AAA");

    printStr(test5);
    test5.insert(1, inserter);
    printStr(test5);
    test5.insert(5,'C');
    printStr(test5);
    test5.insert(7,"BBBBBBBBBB");
    printStr(test5);

    std::cout << "\nTesting appends\n";

    my_str_t adder("111");
    my_str_t test6("____");

    printStr(test6);
    test6.append(adder);
    printStr(test6);
    test6.append('2');
    printStr(test6);
    test6.append("3333333333");
    printStr(test6);
    std::cout << "\nTesting erase\n";
    std::cout << "Erasing first 3 first elements from previous string\n";

    test6.erase(0,3);
    printStr(test6);
    test6.erase(0,20);
    std::cout << "Erased full string, expecting empty string\n";
    printStr(test6);

    std::cout << "\nTesting c_str()\n";
    my_str_t test7("Printing this string using c_str()");
    std::cout << test7.c_str() <<'\n';

    std::cout << "\nTesting finds\n";
    my_str_t test8("String # where we search");
    printStr(test8);
    std::string weStr("we");
    std::cout << "Index of char '#': " << test8.find('#') << '\n';
    std::cout << "Index of std::string 'we': " << test8.find(weStr) << '\n';
    std::cout << "Index of C-like string 'where': " << test8.find("where") << '\n';

    std::cout << "\nTesting substr\n";
    my_str_t test9("0123456789");
    printStr(test9);
    std::cout << "Getting substring from index 1 to 7\n";
    std::cout << test9.substr(1,7);
    //  tests


    //  tests
    my_str_t miwa1("MiwaB");
    my_str_t miwa2("MiwaA");
    char miwa3[] = "MiwaC";

    std::cout << "\n\nComparing\n" << miwa1 << " and " <<  miwa2 << '\n';// testing ==
    std::cout << "Testing '==' operator:\n";
    std::cout << miwa1 << "==" << miwa2 <<"\n" <<(miwa1 == miwa2) << '\n';// testing ==
    std::cout << "Testing '!=' operator:\n";
    std::cout << miwa1 << "!=" << miwa2 <<"\n" << (miwa1 != miwa2) << '\n';// testing !=
    std::cout << "Testing '>' operator:\n";
    std::cout << miwa1 << ">" << miwa2 <<"\n"  << (miwa1 > miwa2) << '\n';// testing >
    std::cout << "Testing '<' operator:\n";
    std::cout << miwa1 << "<" << miwa2 <<"\n" << (miwa1 < miwa2) << '\n';// testing <
    std::cout << "Testing '<=' operator:\n";
    std::cout << miwa1 << "<=" << miwa2 <<"\n"  << (miwa1 <= miwa2) << '\n';// testing <=
    std::cout << "Testing '>=' operator:\n";
    std::cout <<miwa1 << ">=" << miwa2<<"\n"   << (miwa1 >= miwa2) << '\n';// testing >=

    std::cout << "\ncomparing\n" << miwa1 << '\n' <<  miwa3 << '\n';// testing ==
    std::cout << "Testing '==' operator:\n";
    std::cout << miwa1 << "==" << miwa3 <<"\n" <<(miwa1 == miwa3) << '\n';// testing ==
    std::cout << "Testing '!=' operator:\n";
    std::cout << miwa1 << "!=" << miwa3 <<"\n" << (miwa1 != miwa3) << '\n';// testing !=
    std::cout << "Testing '>' operator:\n";
    std::cout << miwa1 << ">" << miwa3 <<"\n"  << (miwa1 > miwa3) << '\n';// testing >
    std::cout << "Testing '<' operator:\n";
    std::cout << miwa1 << "<" << miwa3 <<"\n" << (miwa1 < miwa3) << '\n';// testing <
    std::cout << "Testing '<=' operator:\n";
    std::cout << miwa1 << "<=" << miwa3 <<"\n"  << (miwa1 <= miwa3) << '\n';// testing <=
    std::cout << "Testing '>=' operator:\n";
    std::cout <<miwa1 << ">=" << miwa3<<"\n"   << (miwa1 >= miwa3) << '\n';// testing >=

    std::cout << "\ncomparing\n" << miwa3<<"C-string" << '\n' <<  miwa1 << '\n';// testing ==
    std::cout << "Testing '==' operator:\n";
    std::cout << miwa3 << "==" << miwa1 <<"\n" <<(miwa3 == miwa1) << '\n';// testing ==
    std::cout << "Testing '!=' operator:\n";
    std::cout << miwa3 << "!=" << miwa1 <<"\n" << (miwa3 != miwa1) << '\n';// testing !=
    std::cout << "Testing '>' operator:\n";
    std::cout << miwa3 << ">" << miwa1 <<"\n"  << (miwa3 > miwa1) << '\n';// testing >
    std::cout << "Testing '<' operator:\n";
    std::cout << miwa3 << "<" << miwa1 <<"\n" << (miwa3 < miwa1) << '\n';// testing <
    std::cout << "Testing '<=' operator:\n";
    std::cout << miwa3 << "<=" << miwa1 <<"\n"  << (miwa3 <= miwa1) << '\n';// testing <=
    std::cout << "Testing '>=' operator:\n";
    std::cout <<miwa3 << ">=" << miwa1<<"\n"   << (miwa3 >= miwa1) << '\n';

   std::string path = "/testReadLine.txt";
   testReadline(path);

    std::string pathin = "/Read1word.txt";
    std::cout<<testCin(pathin);

    return 0;
}

my_str_t testCin(const std::string& pathIn){
    std::ifstream inputFile("../"+pathIn);
    my_str_t myStr("");
    inputFile >> myStr;
    inputFile.close();
    return myStr;
}

my_str_t testReadline(std::string path){
    std::ifstream inputFile("../"+path); // Open the input file

    my_str_t myStr("");

    while (readline(inputFile, myStr)) {
        // Process each line read from the file
        std::cout << myStr << std::endl;
    }

    if (inputFile.bad()) {
        std::cerr << "Error reading file!" << std::endl;
    }

    inputFile.close();
    return myStr;
}
void printStr(const my_str_t& str){
    std::cout << str << "| Capacity: " << str.capacity() << " Size: " << str.size() << std::endl;
}
// testing ci on ppush
