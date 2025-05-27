#include "differential_equations_dynamics.hpp"
#include "initial.hpp"    // u1…u12, l1…lv, ai の extern 宣言
#include <cmath>


double f0 (const std::vector<double>& x) { 
    return 1.0; 
}

double f1 (const std::vector<double>& x) { 
    return u1 * std::cos(x[3]); 
}

double f2 (const std::vector<double>& x) { 
    return u1 * std::sin(x[3]); 
}

//theta
double f3 (const std::vector<double>&x) { 
    return u1 * std::tan(x[4])/lv; 
}

//phi
double f4 (const std::vector<double>&x) {  
    return u2; 
}

double f5 (const std::vector<double>& x) { 
    return 1.0; 
}

double f6(const std::vector<double>& x) {
    // x = [t,x,y,theta,phi, x_d,y_d,theta_d,phi_d]
    double u1_dot =  0.0;
    double theta   = x[3];
    double theta_d = x_d[3];
    return u1_dot * std::cos(theta)
         - u1    * std::sin(theta) * theta_d;
}

double f7(const std::vector<double>& x) {
    double u1_dot =  0.0;
    double theta   = x[3];
    double theta_d = x_d[3];
    return u1_dot * std::sin(theta)
         + u1    * std::cos(theta) * theta_d;
}

double f8(const std::vector<double>& x) {
    double u1_dot = 0.0;
    double phi     = x[4];
    double phi_d   = x_d[4];
    return (u1_dot * std::tan(phi)
          + u1    * (phi_d/(std::cos(phi)*std::cos(phi)))) / lv;
}

double f9(const std::vector<double>& x) {
    // phi_d の微分 = u2 の時間微分
    double u2_dot =  0.0;
    return u2_dot;
}


const std::array<FunctionPtr, 5> fAll = {{
  f0, f1, f2, f3, f4
}};
const std::array<FunctionPtr, 5> fdAll = {{
  f5, f6, f7, f8, f9
}};
