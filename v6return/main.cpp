//结论：当一个函数返回{}的时候，会使用函数定义的返回类型的默认数值。
#include <iostream>

struct shit{
	int flag = -1;
};

using namespace std;

struct shit func(int x){
	if (x == x)
	{
		cout << "reaches here!" << endl;
		return {}; //真他娘的牛逼！返回这个““{}”东西，表示使用函数返回类型struct shit的默认数值...
	}
	struct shit y;
	y.flag=0;
	return y;
}

int main(){
	struct shit a = func(142857);
	cout << a. flag << endl;

	return 0;
}






