#define _CRT_SECURE_NO_WARNINGS


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#define N 15
#define Q 100001
#define	DIM	4
#define PAI 3.14159
#define L 1.0

typedef double (*FUNC)(double*);



double	f0(double*), f1(double*), f2(double*), f3(double*), f4(double*);

double V1(double*, int);
double V2(double*, int);



void	initial(double*, double*, double*);

typedef struct {
	double d;
	double Cs;
	double Cs1;
	double Cs2;
	int j;
	double Psx;
	double Psy;

}Search;

Search sr;



static FUNC f[DIM + 1] = { f0, f1, f2, f3, f4 };

Search searchP(double*);
Search searchPP(double*);



static double cs[Q][3];
static double R[Q][2];
static double dRdq[Q][2];
static double d2Rdq2[Q][2];
static double d3Rdq3[Q][2];
static double d4Rdq4[Q][2];




double u1 = 0.5;
double u2;


double v1 = 0.5;
double v2 = 0.5;

double Bx[N + 1] = { -7.0 ,-7.0, -7.0, -7.0, -7.0,-5.5, -4.0, -2.5, -1.0, 0.5, 2.0, -1.0, 0.5, 2.0, 3.5, 5.0 };
double By[N + 1] = { 0.0 ,-1.0, -2.0, -3.0, -7.0, -7.0, -7.0, -7.0, -7.0, -7.0, -7.0, 2.0, 2.0, 2.0, 2.0, 2.0 };

//double Bx[N + 1] = { -7.0 ,-6.0, -5.0, -4.0, -3.0,-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0 };
//double By[N + 1] = { 0.0 , 0.0,  0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };



double qs[Q];





//?K???v?Z
long long int factorial(int n) {
	long long int ans;

	ans = 1;
	for (int i = 2; i <= n; i++) {
		ans *= i;
	}
	return(ans);
}

//??Z
double Power(double a, double b) {
	return(pow(a, b));
}



//?x?W?F???
double Rx(double* b, double* q, int j) {
	int i;
	double ret = 0.0;
	double temp = 0.0;

	for (i = 0; i <= N; i++) {
		ret += b[i] * (factorial(N) / (factorial(N - i) * factorial(i))) * pow(q[j], i) * pow((1 - q[j]), N - i);
		temp += (factorial(N) / (factorial(N - i) * factorial(i)));
		//printf("%.7lf\n", temp);
	}
	return(ret);
}

double Ry(double* b, double* q, int j) {
	int i;
	double ret = 0.0;

	for (i = 0; i <= N; i++) {
		ret += b[i] * (factorial(N) / (factorial(N - i) * factorial(i))) * pow(q[j], i) * pow((1 - q[j]), N - i);

	}
	return(ret);
}

//?x?W?F?????1?K????
double d1Rxdq1(double* b, double* q, int j) {
	int i;
	double ret = 0.0;

	for (i = 1; i <= N; i++) {
		ret += (-b[i - 1] + b[i]) * (factorial(N) / (factorial(N - i) * factorial(i))) * i * pow(q[j], i - 1) * pow((1 - q[j]), N - i);

	}
	return(ret);
}


double d1Rydq1(double* b, double* q, int j) {
	int i;
	double ret = 0.0;

	for (i = 1; i <= N; i++) {
		ret += (-b[i - 1] + b[i]) * (factorial(N) / (factorial(N - i) * factorial(i))) * i * pow(q[j], i - 1) * pow((1 - q[j]), N - i);

	}
	return(ret);
}


//?x?W?F?????2?K????
double d2Rxdq2(double* b, double* q, int j) {
	int i;
	double ret = 0.0;

	for (i = 2; i <= N; i++) {
		ret += (b[i] - 2 * b[i - 1] + b[i - 2])
			* (factorial(N) / (factorial(N - i) * factorial(i))) * i * (i - 1) * pow(q[j], i - 2) * pow((1 - q[j]), N - i);

	}
	return(ret);
}

double d2Rydq2(double* b, double* q, int j) {
	int i;
	double ret = 0.0;

	for (i = 2; i <= N; i++) {
		ret += (b[i] - 2 * b[i - 1] + b[i - 2])
			* (factorial(N) / (factorial(N - i) * factorial(i))) * i * (i - 1) * pow(q[j], i - 2) * pow((1 - q[j]), N - i);

	}
	return(ret);
}

//?x?W?F?????3?K????
double d3Rxdq3(double* b, double* q, int j) {
	int i;
	double ret = 0.0;

	for (i = 3; i <= N; i++) {
		ret += (b[i] - 3 * b[i - 1] + 3 * b[i - 2] - b[i - 3])
			* (factorial(N) / (factorial(N - i) * factorial(i))) * i * (i - 1) * (i - 2) * pow(q[j], i - 3) * pow((1 - q[j]), N - i);

	}
	return(ret);
}

double d3Rydq3(double* b, double* q, int j) {
	int i;
	double ret = 0.0;

	for (i = 3; i <= N; i++) {
		ret += (b[i] - 3 * b[i - 1] + 3 * b[i - 2] - b[i - 3])
			* (factorial(N) / (factorial(N - i) * factorial(i))) * i * (i - 1) * (i - 2) * pow(q[j], i - 3) * pow((1 - q[j]), N - i);

	}
	return(ret);
}


//?x?W?F?????4?K????
double d4Rxdq4(double* b, double* q, int j) {
	int i;
	double ret = 0.0;

	for (i = 4; i <= N; i++) {
		ret += (b[i] - 4 * b[i - 1] + 6 * b[i - 2] - 4 * b[i - 3] + b[i - 4])
			* (factorial(N) / (factorial(N - i) * factorial(i))) * i * (i - 1) * (i - 2) * (i - 3) * pow(q[j], i - 4) * pow((1 - q[j]), N - i);

	}
	return(ret);
}

double d4Rydq4(double* b, double* q, int j) {
	int i;
	double ret = 0.0;

	for (i = 4; i <= N; i++) {
		ret += (b[i] - 4 * b[i - 1] + 6 * b[i - 2] - 4 * b[i - 3] + b[i - 4])
			* (factorial(N) / (factorial(N - i) * factorial(i))) * i * (i - 1) * (i - 2) * (i - 3) * pow(q[j], i - 4) * pow((1 - q[j]), N - i);

	}
	return(ret);
}

//?m?????v?Z
double norm(double* bx, double* by, double* q, int j) {
	double ret = 0.0;
	ret = sqrt(pow(d1Rxdq1(Bx, q, j), 2) + pow(d1Rydq1(By, q, j), 2));

	return(ret);
}

/*????v?Z
double dot(double* bx,double* by, double* q, int j, double x, double y) {
	double ret = 0.0;

	ret = x * norm(bx,by, q, j) + y * norm(bx,by, q, j);

	return(ret);
}
*/




int main()
{
	static double	k[DIM + 1][4], q[DIM + 1][4], r[DIM + 1][4];
	static double	x[3][DIM + 1];
	static double	x_old[DIM + 1], x_new[DIM + 1];




	FILE* fout;




	double	h, t_max;
	long	i, j, n, time;

	qs[0] = 0.0;

	//?o?H??Q????
	for (i = 1; i < Q; i++) {
		qs[i] = qs[i - 1] + 0.00001;
	}

	for (i = 0; i < Q; i++) {

		R[i][0] = Rx(Bx, qs, i);
		R[i][1] = Ry(By, qs, i);
		dRdq[i][0] = d1Rxdq1(Bx, qs, i);
		dRdq[i][1] = d1Rydq1(By, qs, i);

		cs[i][0] = (-(d1Rydq1(By, qs, i) * d2Rxdq2(Bx, qs, i)) + d1Rxdq1(Bx, qs, i) * d2Rydq2(By, qs, i)) /
			Power(Power(d1Rxdq1(Bx, qs, i), 2) + Power(d1Rydq1(By, qs, i), 2), 1.5);


		cs[i][1] = (Power(d1Rydq1(By, qs, i), 2) * (3 * d2Rxdq2(Bx, qs, i) * d2Rydq2(By, qs, i) - d1Rydq1(By, qs, i) * d3Rxdq3(Bx, qs, i))
			- Power(d1Rxdq1(Bx, qs, i), 2) * (3 * d2Rxdq2(Bx, qs, i) * d2Rydq2(By, qs, i) + d1Rydq1(By, qs, i) * d3Rxdq3(Bx, qs, i))
			+ Power(d1Rxdq1(Bx, qs, i), 3) * d3Rydq3(By, qs, i) + d1Rxdq1(Bx, qs, i) * d1Rydq1(By, qs, i) * (3 * Power(d2Rxdq2(Bx, qs, i), 2)
				- 3 * Power(d2Rydq2(By, qs, i), 2) + d1Rydq1(By, qs, i) * d3Rydq3(By, qs, i))) / Power(Power(d1Rxdq1(Bx, qs, i), 2) + Power(d1Rydq1(By, qs, i), 2), 3);


		cs[i][2] = (-(Power(d1Rxdq1(Bx, qs, i), 4) * (4 * d2Rydq2(By, qs, i) * d3Rxdq3(Bx, qs, i)
			+ 6 * d2Rxdq2(Bx, qs, i) * d3Rydq3(By, qs, i) + d1Rydq1(By, qs, i) * d4Rxdq4(Bx, qs, i)))
			+ Power(d1Rxdq1(Bx, qs, i), 2) * d1Rydq1(By, qs, i) * (-15 * Power(d2Rxdq2(Bx, qs, i), 3)
				+ d2Rxdq2(Bx, qs, i) * (39 * Power(d2Rydq2(By, qs, i), 2) - 2 * d1Rydq1(By, qs, i) * d3Rydq3(By, qs, i))
				+ 2 * d1Rydq1(By, qs, i) * (d2Rydq2(By, qs, i) * d3Rxdq3(Bx, qs, i) - d1Rydq1(By, qs, i) * d4Rxdq4(Bx, qs, i)))
			+ Power(d1Rydq1(By, qs, i), 3) * (3 * Power(d2Rxdq2(Bx, qs, i), 3) + d2Rxdq2(Bx, qs, i) * (-15 * Power(d2Rydq2(By, qs, i), 2)
				+ 4 * d1Rydq1(By, qs, i) * d3Rydq3(By, qs, i)) + d1Rydq1(By, qs, i) * (6 * d2Rydq2(By, qs, i) * d3Rxdq3(Bx, qs, i)
					- d1Rydq1(By, qs, i) * d4Rxdq4(Bx, qs, i))) + Power(d1Rxdq1(Bx, qs, i), 5) * d4Rydq4(By, qs, i)
			+ d1Rxdq1(Bx, qs, i) * Power(d1Rydq1(By, qs, i), 2) * (-39 * Power(d2Rxdq2(Bx, qs, i), 2) * d2Rydq2(By, qs, i)
				+ 15 * Power(d2Rydq2(By, qs, i), 3) + 10 * d1Rydq1(By, qs, i) * d2Rxdq2(Bx, qs, i) * d3Rxdq3(Bx, qs, i)
				- 10 * d1Rydq1(By, qs, i) * d2Rydq2(By, qs, i) * d3Rydq3(By, qs, i) + Power(d1Rydq1(By, qs, i), 2) * d4Rydq4(By, qs, i))
			+ Power(d1Rxdq1(Bx, qs, i), 3) * (15 * Power(d2Rxdq2(Bx, qs, i), 2) * d2Rydq2(By, qs, i) - 3 * Power(d2Rydq2(By, qs, i), 3)
				+ 10 * d1Rydq1(By, qs, i) * d2Rxdq2(Bx, qs, i) * d3Rxdq3(Bx, qs, i) - 10 * d1Rydq1(By, qs, i) * d2Rydq2(By, qs, i) * d3Rydq3(By, qs, i)
				+ 2 * Power(d1Rydq1(By, qs, i), 2) * d4Rydq4(By, qs, i))) / Power(Power(d1Rxdq1(Bx, qs, i), 2) + Power(d1Rydq1(By, qs, i), 2), 4.5);

	}


	fout = fopen("ex9cs.csv", "w");

	for (i = 0; i < Q; i++) {
		fprintf(fout, "%1.7lf,", qs[i]);
		fprintf(fout, "%1.7lf,", R[i][0]);
		fprintf(fout, "%1.7lf,", R[i][1]);
		fprintf(fout, "\n");
	}


	fout = fopen("ex9.csv", "w");

	initial(&t_max, &h, x_old);


	n = (long)(t_max / h) + 1;

	for (i = 0; i < DIM + 1; i++) {
		fprintf(fout, "%10.7lf,", x_old[i]);
	}

	//?S?T??
	searchP(x_old);
	fprintf(fout, "%10.7lf,", sr.Psx);
	fprintf(fout, "%10.7lf,", sr.Psy);
	fprintf(fout, "%10.7lf,", sr.d);
	fprintf(fout, "%10d,", sr.j);
	fprintf(fout, "\n");



	//?????Q?N?b?^?M???@
	for (i = 0; i < DIM + 1; i++)
		q[i][3] = 0.0;
	for (j = 1; j < n; j++) {

		if (j % 100 == 0) {
			printf("%d\n", j);
		}
		

		//???x?v?Z
		V1(x_old, sr.j);
		V2(x_old, sr.j);

		for (i = 0; i < DIM + 1; i++) {
			k[i][0] = h * (*f[i])(x_old);
			r[i][0] = (k[i][0] - 2.0 * q[i][3]) / 2.0;
			x[0][i] = x_old[i] + r[i][0];
			q[i][0] = q[i][3] + 3.0 * r[i][0] - k[i][0] / 2.0;
		}
		for (i = 0; i < DIM + 1; i++) {
			k[i][1] = h * (*f[i])(x[0]);
			r[i][1] = (1.0 - sqrt(0.5)) * (k[i][1] - q[i][0]);
			x[1][i] = x[0][i] + r[i][1];
			q[i][1] = q[i][0] + 3.0 * r[i][1] - (1.0 - sqrt(0.5)) * k[i][1];
		}
		for (i = 0; i < DIM + 1; i++) {
			k[i][2] = h * (*f[i])(x[0]);
			r[i][2] = (1.0 + sqrt(0.5)) * (k[i][2] - q[i][1]);
			x[2][i] = x[1][i] + r[i][2];
			q[i][2] = q[i][1] + 3.0 * r[i][2] - (1.0 + sqrt(0.5)) * k[i][2];
		}
		for (i = 0; i < DIM + 1; i++) {
			k[i][3] = h * (*f[i])(x[0]);
			r[i][3] = (k[i][3] - 2.0 * q[i][2]) / 6.0;
			x_new[i] = x[2][i] + r[i][3];
			q[i][3] = q[i][2] + 3.0 * r[i][3] - k[i][3] / 2.0;
		}



		for (i = 0; i < DIM + 1; i++) {
			fprintf(fout, "%10.7lf,", x_new[i]);

		}
		fprintf(fout, "%10.7lf,", sr.Psx);
		fprintf(fout, "%10.7lf,", sr.Psy);
		fprintf(fout, "%10.7lf,", sr.d);
		fprintf(fout, "%10d,", sr.j);
		fprintf(fout, "\n");


		for (i = 0; i < DIM + 1; i++)
			x_old[i] = x_new[i];

		//?????T??
		searchPP(x_old);
	}

	return 0;


}

void initial(double* t_max, double* dt, double* x0) {



	fflush(stdin);

	*t_max = 50.0;

	*dt = 0.01;


	x0[1] = -6.001;
	x0[2] = 1.0;
	x0[3] = -PAI / 2.0;
	x0[4] = -PAI / 4.0;


}


double f0(double* x) {

	return(1.0);
}

double f1(double* x) {
	/*??????????x*/
	double	t = x[0];
	double	ret;
	ret = v1 * cos(x[3]);
	return(ret);
}

double f2(double* x) {
	/*y????????x*/
	double	t = x[0];
	double	ret;
	ret = v1 * sin(x[3]);

	return(ret);
}

double f3(double* x) {

	double	t = x[0];
	double	ret;
	ret = v1 * tan(x[4]);
	return(ret);
}

double f4(double* x) {

	double	t = x[0];
	double	ret;


	ret = v2;
	return(ret);
}

//Ps?T??
Search searchP(double* x) {


	int i;
	double dot = 0.0;
	double dist = 0.0;
	double dist0 = DBL_MAX;


	for (i = 0; i < Q; i++) {


		//????v?Z
		dot = (x[1] - R[i][0]) * (dRdq[i][0] / norm(Bx, By, qs, i)) + (x[2] - R[i][1]) * (dRdq[i][1] / norm(Bx, By, qs, i));

		dist = sqrt(pow((x[1] - R[i][0]), 2) + pow((x[2] - R[i][1]), 2));

		if (-0.0001 < dot && dot < 0.0001) {

			if (dist < dist0) {
				dist0 = dist;
				sr.Psx = R[i][0];
				sr.Psy = R[i][1];
				sr.d = (x[1] - R[i][0]) * (-(dRdq[i][1] / norm(Bx, By, qs, i))) + (x[2] - R[i][1]) * (dRdq[i][0] / norm(Bx, By, qs, i));
				sr.Cs = cs[i][0];
				sr.Cs1 = cs[i][1];
				sr.Cs2 = cs[i][2];
				sr.j = i;

			}

		}
	}

	return(sr);

}

Search searchPP(double* x) {
	int i;
	double dot = 0.0;
	double dist = 0.0;
	double dist0= DBL_MAX;

	if (sr.j < 400) {
		for (i = 0; i < sr.j + 400; i++) {
			//????v?Z
			dot = (x[1] - R[i][0]) * (dRdq[i][0] / norm(Bx, By, qs, i)) + (x[2] - R[i][1]) * (dRdq[i][1] / norm(Bx, By, qs, i));

			dist = sqrt(pow((x[1] - R[i][0]), 2) + pow((x[2] - R[i][1]), 2));

			if (-0.0001 < dot && dot < 0.0001) {

				if (dist < dist0) {
					dist0 = dist;
					sr.Psx = R[i][0];
					sr.Psy = R[i][1];
					sr.d = (x[1] - R[i][0]) * (-(dRdq[i][1] / norm(Bx, By, qs, i))) + (x[2] - R[i][1]) * (dRdq[i][0] / norm(Bx, By, qs, i));
					sr.Cs = cs[i][0];
					sr.Cs1 = cs[i][1];
					sr.Cs2 = cs[i][2];
					sr.j = i;
				}

		}
	}}else if (sr.j > Q - 400) {
		for (i = sr.j - 400; i < Q; i++) {
			//????v?Z
			dot = (x[1] - R[i][0]) * (dRdq[i][0] / norm(Bx, By, qs, i)) + (x[2] - R[i][1]) * (dRdq[i][1] / norm(Bx, By, qs, i));

			dist = sqrt(pow((x[1] - R[i][0]), 2) + pow((x[2] - R[i][1]), 2));

			if (-0.0001 < dot && dot < 0.0001) {

				if (dist < dist0) {
					dist0 = dist;
					sr.Psx = R[i][0];
					sr.Psy = R[i][1];
					sr.d = (x[1] - R[i][0]) * (-(dRdq[i][1] / norm(Bx, By, qs, i))) + (x[2] - R[i][1]) * (dRdq[i][0] / norm(Bx, By, qs, i));
					sr.Cs = cs[i][0];
					sr.Cs1 = cs[i][1];
					sr.Cs2 = cs[i][2];
					sr.j = i;
				}

			}

		}
	}
	else {
		for (i = sr.j - 400; i < sr.j+400; i++) {
			//????v?Z
			dot = (x[1] - R[i][0]) * (dRdq[i][0] / norm(Bx, By, qs, i)) + (x[2] - R[i][1]) * (dRdq[i][1] / norm(Bx, By, qs, i));

			dist = sqrt(pow((x[1] - R[i][0]), 2) + pow((x[2] - R[i][1]), 2));

			if (-0.0001 < dot && dot < 0.0001) {

				if (dist < dist0) {
					dist0 = dist;
					sr.Psx = R[i][0];
					sr.Psy = R[i][1];
					sr.d = (x[1] - R[i][0]) * (-(dRdq[i][1] / norm(Bx, By, qs, i))) + (x[2] - R[i][1]) * (dRdq[i][0] / norm(Bx, By, qs, i));
					sr.Cs = cs[i][0];
					sr.Cs1 = cs[i][1];
					sr.Cs2 = cs[i][2];
					sr.j = i;
				}

			}

		}
	}


	return(sr);

}

//v1?v?Z
double V1(double* x, int i) {


	v1 = ((1 - sr.d * sr.Cs) / cos(x[3] - atan2(dRdq[i][1], dRdq[i][0]))) * u1;


	return(v1);
}



//v2?v?Z
double V2(double* x, int i) {

	double dx2ds = 0.0;
	double dx2dd = 0.0;
	double dx2dthp = 0.0;
	double dx2dphi = 0.0;
	double a1;
	double a2;

	double thp = x[3] - atan2(dRdq[i][1], dRdq[i][0]);

	//??L?l?v?Z
	double z1 = -sr.Cs1 * sr.d * tan(thp)
		- sr.Cs * (1 - sr.d * sr.Cs) * ((1 + pow(sin(thp), 2)) / pow(cos(thp), 2))
		+ pow((1 - sr.d * sr.Cs), 2) * tan(x[4]) / (L * pow(cos(thp), 3));
	double z2 = ((1 - sr.d * sr.Cs) * tan(thp)) / u1;
	double z3 = sr.d / pow(u1, 2);
	double a = -1.5;

	double p1, p2, p3;

	p1 = 3 * a;
	p2 = -3 * a * a;
	p3 = a * a * a;

	u2 = p1 * z1 + p2 * z2 + p3 * z3;







	dx2ds = -sr.Cs2 * sr.d * tan(thp)
		- sr.Cs1 * (1 - sr.d * sr.Cs) * ((1 + pow(sin(thp), 2)) / pow(cos(thp), 2))
		+ sr.d * sr.Cs * sr.Cs1 * ((1 + pow(sin(thp), 2))) / pow(cos(thp), 2)
		- sr.d * sr.Cs1 * (2 * (1 - sr.d * sr.Cs) * tan(x[4])) / (L * pow(cos(thp), 3));

	dx2dd = -sr.Cs1 * tan(x[3] - atan2(dRdq[i][1], dRdq[i][0]))
		+ sr.Cs * sr.Cs * ((1 + pow(sin(thp), 2)) / pow(cos(thp), 2))
		- (2 * (1 - sr.d * sr.Cs) * tan(x[4]) * sr.Cs) / (L * pow(cos(thp), 3));


	dx2dthp = -sr.Cs1 * sr.d / pow(cos(thp), 2)
		- sr.Cs * (1 - sr.d * sr.Cs) * 4 * sin(thp) / pow(cos(thp), 3)
		+ 3 * (pow((1 - sr.d * sr.Cs), 2) * tan(x[4]) * sin(thp)) / (L * pow(cos(thp), 4));

	dx2dphi = pow((1 - sr.d * sr.Cs), 2) / (L * pow(cos(thp), 3) * pow(cos(x[4]), 2));

	a1 = dx2ds + dx2dd * (1 - sr.d * sr.Cs) * tan(thp)
		+ dx2dthp * ((tan(x[4]) * (1 - sr.d * sr.Cs)) / (L * cos(thp)) - sr.Cs);

	a2 = (L * pow(cos(thp), 3) * pow(cos(x[4]), 2)) / pow((1 - sr.d * sr.Cs), 2);

	/* u2 = dx2ds * 0.5
		 + dx2dd * sin(x[3] - atan2(dRdq[i][1], dRdq[i][0])) * v1
		 + dx2dthp * (v1 * tan(x[4]) / L - (v1 * sr.Cs * cos(x[3] - atan2(dRdq[i][1], dRdq[i][0]))) / (1 - sr.d * sr.Cs))
		 + dx2dphi * v2;*/



	v2 = a2 * (u2 - a1 * u1);

	return(v2);
}







