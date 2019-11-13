#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <stdio.h>

int main(){
    DIR *dir;
    struct dirent *ptr;
    dir = opendir("."); ///open the dir

    while((ptr = readdir(dir)) != NULL) ///read the list of this dir
    {
        #ifdef _WIN32
            printf("d_name: %s\n", ptr->d_name);
        #endif
        #ifdef __linux
            printf("d_type:%d d_name: %s\n", ptr->d_type,ptr->d_name);
        #endif
    }
    closedir(dir);
    return 0;
}
