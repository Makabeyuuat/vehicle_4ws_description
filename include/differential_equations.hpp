#ifndef DIFFERENTIAL_EQUATIONS_HPP
#define DIFFERENTIAL_EQUATIONS_HPP

#include <vector>
#include <cmath>
#include <array>

// 状態ベクトルの次元
extern const int DIM;

// 各状態の微分方程式のシグネチャ
double f0 (const std::vector<double>& x);
double f1 (const std::vector<double>& x);
double f2 (const std::vector<double>& x);
double f3 (const std::vector<double>& x);
double f4 (const std::vector<double>& x);

// 関数ポインタ型エイリアス（getInputValue からも使えます）
using FunctionPtr = double(*)(const std::vector<double>&);
extern const std::array<FunctionPtr, /*DIM+1=*/5> fAll;

#endif // DIFFERENTIAL_EQUATIONS_HPP
