#include <iostream>
#include <vector>


using namespace std;
int main(){
	vector<int> a = {1,4,2,8,5,7};
	for (auto &i : a)  //vector<int>{1,2,3}
	{
		cout << i << " ";
	}
	cout << endl;
	//a.clear();
	for (auto& i: a) cout << i << " " << endl;
	cout << "Number of elements in a is " << a.size() << endl;
	a.push_back(9);
	for (auto& i: a) cout << i << " " << endl;
	return 0;
}







