#include <iostream>
#include <vector>
#include <algorithm>
#include<numeric>

/*
结论：测试vector的求和以及求均值方法
*/
using namespace std;
int main()
{
	//DWORD start = ::GetTickCount();
	vector<double> a = {1.,4,2,8,5,7};
	//所有元素相加accumulate中的函数参数
	cout << "vector's sum is " << accumulate(begin(a) + 2, end(a), 0.0) << endl;
	cout << "vector's mean is " << accumulate(begin(a), end(a), 0.0)/a.size() << endl;
	//cout << ::GetTickCount() - start << " ms" << endl;
	return 0;
}






