#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

#include "my_directory_operators.h"


using namespace std;
int main(){
    const char* path = ".";
    auto fn = get_filenames_in_directory(path, 2);
    for (auto &i:fn){
        cout << i << endl;
    }
    return 0;
}





