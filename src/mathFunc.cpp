#include <stdio.h>
#include <math.h>

#include "mathFunc.h"



//三角関数
double Cos(double x) {
	return(cos(x));
}

double Sin(double x) {
	return(sin(x));
}

double Tan(double x) {
	return(tan(x));
}

double ArcTan(double  x, double y) {
	return atan2(y, x);
}

double Sec(double x) {
	return(1 / cos(x));
}

//階乗
long long int factorial(double n) {
	long long int ans;

	ans = 1;
	for (int i = 2; i <= n; i++) {
		ans *= i;
	}
	return(ans);
}

double Power(double x, double y) {

	return(pow(x, y));
}

double Sqrt(double x) {
	return(sqrt(x));
}

