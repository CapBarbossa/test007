#include <sys/time.h>
#include <iostream>


using namespace std;
int main()
{
	timespec t1, t2;
	clock_gettime(CLOCK_MONOTONIC, &t1);
	int sum = 0;
	for (int i = 0; i < 1e8; i++)
	{
		sum += 1;
	}
	clock_gettime(CLOCK_MONOTONIC, &t2);
	auto delta_nm = (t2.tv_sec - t1.tv_sec) * 10^9 + t2.tv_nsec - t1.tv_nsec;   //纳秒计时
	cout << delta_nm << " ns" << endl;
	//微秒
	struct timeval t3, t4;
	gettimeofday(&t3, NULL);
	for (int i = 0; i < 1e8; i++) sum += 1;
	gettimeofday(&t4, NULL);
	auto delta_um = (t4.tv_sec - t3.tv_sec) * 1e6 + t4.tv_usec - t3.tv_usec;
	cout << delta_um << " us." << endl;
	cout << delta_um / 1e3 << " ms." << endl;
	return 0;
}

