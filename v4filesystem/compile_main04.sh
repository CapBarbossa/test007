#!/bin/bash
### 教训: 编译命令中文件的顺序可能会导致编译失败，在这里，main04.cpp必须在so文件的位置之前.
g++ -std=c++11 main04.cpp -I incubator01/ -L incubator01/ -lmyfilenames -o main04
