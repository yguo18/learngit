CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(hello) #项目名称
#把当前目录(.)下所有源代码文件和头文件加入变量SRC_LIST
AUX_SOURCE_DIRECTORY(. SRC_LIST)
#生成应用程序 hello (在windows下会自动生成hello.exe)
ADD_EXECUTABLE(hello ${SRC_LIST})
#cmake_minimum_required (VERSION 2.6)
#project (Tutorial)   # 项目名
#add_executable(Tutorial tutorial.cxx)
