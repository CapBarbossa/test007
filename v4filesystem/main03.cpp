#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <iostream>


using namespace std;
int main(){
    DIR *dir;
    struct dirent *ptr;
    dir = opendir("."); ///open the dir

    while((ptr = readdir(dir)) != NULL) ///read the list of this dir
    {
    	if (ptr->d_name == ".") {
    		cout << "kkk" << endl;
    	}
       	printf("d_name: %s\n", ptr->d_name);
        //printf("d_name: %s\n", ptr->d_name);
    }
    closedir(dir);
    return 0;
}
