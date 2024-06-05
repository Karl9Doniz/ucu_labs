# Lab work <mark>2</mark>: <mark>Files Composing</mark>
Authors (team): <mark><br>Bohdan Pavliuk[https://github.com/BohdanPavliuk]
<br>Andrii Ahitoliev[https://github.com/Karl9Doniz]
<br>Igor Ivanyshyn[https://github.com/xsponenta]</mark><br>
Variant: <mark>2nd Variant(zlib)</mark>

## Prerequisites

<mark>You should have installed: GCC or clang, CMake, and make. Also requires installed Shell</mark>
### Work devision

Igor (CMake file for 2_zlib_sample, additional task 2)<br>
Bohdan (CMake file for mystring; script.sh for 2_zlib_sample; README)<br>
Andrii (script.sh for mystring; make files for both libraries)<br>

### Compilation

<mark>As our lab is about compiling files all compilation work will be shown in Usage</mark>

### Usage

<markdown>First of all you need to clone our git repository. Open your terminal in the directory you want to copy our lab<br>
``` shell
git clone https://github.com/ucu-cs/lab1-cstring-ahitoliev_ivanyshyn_pavliuk
```
Then open mystring directory and firstly change execute permission and run our script
``` shell
cd mystring/
chmod +x script.sh
./script.sh 
```
It will create directory build in mystring/. So open build directory and test our program by next commands 
``` shell
cd build/
./main_dynamic 
./main_static 
```
It have to look like this in both causes 
<img width="757" alt="Screenshot 2023-10-19 at 19 50 05" src="https://github.com/ucu-cs/lab2-make-cmake-ahitoliev_ivanyshyn_pavliuk/assets/55399864/ea4dfcb2-8823-48f0-aaff-4512bc749886"><br>
If it works - Congratulations!!!.<br> Don't forget to delete build directory after usage.<br><br>
Next test CMake. Enter to mystring directory and create new directory. IN our case it's b/ folder. Enter it and run CMake file<br>
```shell
mkdir b/
cd b/
cmake .. && make
```
<img width="680" alt="Screenshot 2023-10-19 at 19 55 45" src="https://github.com/ucu-cs/lab2-make-cmake-ahitoliev_ivanyshyn_pavliuk/assets/55399864/9806c47f-f8a4-4ad3-a853-9e3e8f205aea"><br>
That's how it looks like<br>
To test the project run 

``` shell
./examples/example_dynamic
./examples/example_static 
```
It has to look like previous testing. 
<br><br>
NEXT we will test our 2_zlib_sample. Firstly you have to be in root directory of our lab. Then enter to 2_zlib_sample and run our script<br>
``` shell
cd 2_zlib_sample/
chmod +x script.sh
./script.sh 
```
To test program enter to new created build folder and run <br>
``` shell
./example_dynamic 
./example_static 
```

<img width="441" alt="Screenshot 2023-10-19 at 20 03 08" src="https://github.com/ucu-cs/lab2-make-cmake-ahitoliev_ivanyshyn_pavliuk/assets/55399864/1474fa80-18c2-4648-88ea-c26be39cd8f5"><br>
it tests only example, to test minigzip run next
``` shell
./minigzip_dynamic -h ../test.txt 
./minigzip_dynamic -d ../test.txt.gz
./minigzip_static -h ../test.txt
 ./minigzip_static -d ../test.txt.gz
```
This program compress and decompress you test.txt file, which is in 2_zlib_example
<br><br>
To test CMake create b/ folder in your 2_zlib_sample/ enter it and run cmake command. All code next
``` shell
mkdir b/
cd b/
cmake .. && make
```
<img width="733" alt="Screenshot 2023-10-19 at 20 11 07" src="https://github.com/ucu-cs/lab2-make-cmake-ahitoliev_ivanyshyn_pavliuk/assets/55399864/a502d522-d154-4c53-931d-c52af37fa291"><br>
And test our program
``` shell
./examples/example_dynamic
./examples/example_static
./examples/example_minigzip_dynamic -h ../test.txt
./examples/example_minigzip_dynamic -d ../test.txt.gz
./examples/example_minigzip_static  -h ../test.txt
./examples/example_minigzip_static  -d ../test.txt.gz
```
Testing will look like from script


**Makefiles and their compilation.**
In the **2_zlib_sample** you can find two make files: makefile and makemainfile which should be executed in that same order.
To compile the library .c files do the following:
``` shell
make -f makefile
```
To clean the directories:
``` shell
make -f makefile clean
```
Similarly, to link main files from **example** directory, run makemainfile:
``` shell
make -f makemainfile
```
The snapshots of running makemainfile commands:

<img width="1620" alt="Screenshot 2023-10-19 at 21 54 33" src="https://github.com/ucu-cs/lab2-make-cmake-ahitoliev_ivanyshyn_pavliuk/assets/44242769/73589123-87d8-4a25-a77e-0fc4fe147e2f">

... and makefile:

<img width="1622" alt="Screenshot 2023-10-19 at 21 57 47" src="https://github.com/ucu-cs/lab2-make-cmake-ahitoliev_ivanyshyn_pavliuk/assets/44242769/5aa7233e-ada9-4974-a8d7-c87c4d7d80d7">


We are getting following executable files:

<img width="248" alt="Screenshot 2023-10-19 at 21 58 27" src="https://github.com/ucu-cs/lab2-make-cmake-ahitoliev_ivanyshyn_pavliuk/assets/44242769/123b9ec6-d876-46e9-832e-d4e5addd67d9">

</markdown>


### Important!

<mark>We test 
our Cmake, make, and shell script only in MacOS. It can work unproperly on other systems.<br>
Also, it fails in our string library, because we didn't fix some undefined behavior.<br>
However, it hasn't to influence our compilations  </mark>

### Results

<mark>So we test three ways to compose our files and deal with CMake, make and shell running.<br> 
We find Cmake the most convenient way to compose our files, because it has understandable syntax and<br>
powerful tools </mark>

# Additional tasks

Additional task 2. CMake for searching libraries

<markdown>
First of all you need to create folder b/ in mystring directory and run cmake command
 
``` shell
cd mystring/
mkdir b/
cd b/
cmake .. && make
```
After that exit to root directory and create also folder b/
``` shell
cd ..
cd ..
mkdir b/
cd b/
cmake .. && make
```
It will run CMake file for searching library and print something like this<be>

<img width="1128" alt="Screenshot 2023-10-19 at 21 40 32" src="https://github.com/ucu-cs/lab2-make-cmake-ahitoliev_ivanyshyn_pavliuk/assets/55399864/65cdce01-a555-4dc2-b9be-7ad6c6ca4c70"><br>
and here it shows if it found our library or not
</markdown>


