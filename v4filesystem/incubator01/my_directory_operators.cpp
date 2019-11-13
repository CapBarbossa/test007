//
// Created by bro on 19-6-19.
//
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include "my_directory_operators.h"

std::vector<std::string> get_filenames_in_directory(const char* path, int flag){
    /*
     *path: 目标路径
     * flag: 0-目录 1-文件，2-我全都要
     */
    struct dirent *ptr;
    auto dir = opendir(path);
    std::vector<std::string> goals;

    while((ptr=readdir(dir)) != NULL){
        if (flag == 2){
            goals.push_back(ptr->d_name);
        }
        else if (flag == 0 && ptr->d_type==4){   //目录
            goals.push_back(ptr->d_name);
        }
        else if (flag == 1 && ptr->d_type==8){  //文件
            goals.push_back(ptr->d_name);
        }
    }
    closedir(dir);
    return goals;
}





