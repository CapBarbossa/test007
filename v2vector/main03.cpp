// 本脚本用以练习对vector的元素进行排序

#include <iostream>
#include <vector>
#include "opencv2/highgui/highgui.hpp"    // wtf! function sort is in here.

//#include <algorithm>

using namespace std;
using namespace cv;
int main()
{
	vector<int> a{1,4,2,8,5,7};
	sort(a.rbegin(), a.rend());
	for (auto &i:a){
		cout << i << endl;
	}
	vector<Rect> b;
	b.push_back(Rect(1,2,3,4));
	b.push_back(Rect(2,1,4,2));
	b.push_back(Rect(4,5,6,7));
	sort(b.begin(), b.end(), [](Rect x1, Rect x2){return x1.width * x1.height > x2.width * x2.height;});
	for (auto &j: b){
		cout << j	<< endl;
	}
	
	return 0;
}









