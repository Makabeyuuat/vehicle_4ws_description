#include "differential_equations.hpp"
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



const std::array<FunctionPtr, 5> fAll = {{f0, f1, f2, f3, f4}};
