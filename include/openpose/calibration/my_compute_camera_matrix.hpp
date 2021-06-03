#include <iostream>
#include <math.h>

using namespace std;

class Compute_Camera_Matrix
{
public:

	Compute_Camera_Matrix()
	{
	};

	~Compute_Camera_Matrix()
	{
	};

	Compute_Camera_Matrix(double f1, double f2, double theta, double x, double y, double z)
	{
		const long double PI = 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117067982148086513282306647093844;

		theta = theta * PI / 180.0;

		cout << "====Camera Matrix====" << endl;
		cout << setiosflags(ios::fixed) << setprecision(3) << f1 * cos(theta) << " " << -f1 * sin(theta) << " 0.000 " << f1 * x << endl;
		cout << setiosflags(ios::fixed) << setprecision(3) << f2 * sin(theta) << " " << f2 * cos(theta) << " 0.000 " << f2 * y << endl;
		cout << setiosflags(ios::fixed) << setprecision(3) << "0.000 0.000 1.000 " << z << endl;
	}
};