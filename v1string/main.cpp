#include <iostream>
#include <string>
#include <libgen.h>

using namespace std;

int leng(const string&);
string leng2(string&);
int main()
{
	string a = "/home/MyWork/datas/imgs/20190109.JPEG";
	cout << a.substr(0, a.find_last_of(".")-1) << endl;
	/*
	auto b = a.substr(a.find_last_of("/")+1);
	auto c = basename("/home/bro/456.jpg");
	cout << b << endl;
	cout << c << endl;
	string *d = &a;
	cout << leng(a) << endl;
	cout << leng2(a) << endl;
	auto x = a.find("142857");
	auto y = a.find_last_of("142857");
	if (x != string::npos)
		cout << x << endl;
	string e = "142857";
	string ee = (e+=".png");
	cout << ee << endl;
	*/
	return 0;
}


int leng(const string& f)
{
	return f.find_last_of("/");
}

string leng2(string& f)
{
	f = f.substr(f.find_last_of("/")+1);
	return f;
};
