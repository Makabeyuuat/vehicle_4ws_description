#pragma once
double Rx(double* b, double* q, int j), Ry(double* b, double* q, int j);
double d1Rxdq1(double* b, double* q, int j), d1Rydq1(double* b, double* q, int j);
double d2Rxdq2(double* b, double* q, int j), d2Rydq2(double* b, double* q, int j);
double d3Rxdq3(double* b, double* q, int j), d3Rydq3(double* b, double* q, int j);
double d4Rxdq4(double* b, double* q, int j), d4Rydq4(double* b, double* q, int j);
double norm(double* bx, double* by, double* q, int j);
