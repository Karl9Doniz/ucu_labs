#include <iostream>
#include "Ahitoliev_Pavluk_Ivanyshun_string/mystring.cpp"
#include <fstream>
#include <sstream>

int main(int argc, char **argv) {
    if (argc < 3){
        std::cerr << "3 arguments are required" << std::endl;
    }
    std::string readfile = argv[1];
    std::string writefile = argv[2];

    std::ifstream InputFile(readfile);
    int counter = 0;
    std::string word;
    std::stringstream fileContentsStream;
    if (InputFile.is_open()){
        while (InputFile >> word){
            fileContentsStream << word << '\n';
            counter++;
        }

        InputFile.close();
    }else {
        std::cerr << "Can't open a file" << std::endl;
    }
    my_str_t *words[counter];
    int i = 0;
    while (fileContentsStream >> word){
        words[i] = new my_str_t(word);
        i++;
    }
    int half_counter = counter / 2;
    my_str_t *temp;
    for (int i = 0; i < half_counter; i++){
        temp = words[2 * i];
        temp->append(":");
        temp->append(*words[i * 2 + 1]);
        words[2 * i] = new my_str_t("");
        words[2 * i  + 1] = new my_str_t("");
        words[i] = temp;
    }
    std::ofstream MyFile(writefile);
    my_str_t *empty_str = new my_str_t("");
    for(int i = 0; i < counter; i++){
        if (*words[i] == *empty_str){
            continue;
        }
        MyFile << *words[i] << "\n";
    }
    MyFile.close();
    return 0;
}
