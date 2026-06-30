cmake_minimum_required(VERSION 3.10)
set(project "SCServo")
project(${project})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

file(GLOB hdrs *.h)
file(GLOB srs *.cpp)



add_library(${project} STATIC ${hdrs} ${srs})
#add_executable(${project} main.cpp ${hdrs} ${srs})

# 设置库的包含目录
target_include_directories(${project} 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}  # 使得其他项目可以找到头文件
)
