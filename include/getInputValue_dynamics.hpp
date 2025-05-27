#ifndef GET_INPUT_VALUE_HPP
#define GET_INPUT_VALUE_HPP

#include <vector>
#include <cmath>
#include <array>
#include <functional>


class getInputValue {
public:
    // 各状態の微分方程式右辺関数のポインタ型
    using FunctionPtr = double(*)(const std::vector<double>&);

    // /**
    //  * @brief コンストラクタ
    //  * @param h_ 時間刻み（ステップサイズ）
    //  * @param f_ 各状態に対応する微分関数ポインタの配列（サイズは Dim+1）
    //  */
    getInputValue(double h);

    // /**
    //  * @brief ルンゲ‐クッタ法による状態更新と制御入力計算を実行する
    //  * @param x_old 現在の状態（サイズは Dim+1）
    //  * @param sr_j  制御入力計算用の参照インデックス（例：センサ情報など）
    //  */
    void rungeKutta(std::vector<double>& x_old, std::vector<double>& x_d);
    void ddrungeKutta(std::vector<double>& x_old, std::vector<double>& x_dd);
    void getU(std::vector<double>& x_old, int sr_j);
    void getXInput(std::vector<double>& x_old, std::vector<double>& x_input);
private:
    double h;  // 時間刻み

    // ルンゲ‐クッタ法用中間計算変数（サイズはそれぞれ Dim+1）
    std::vector<std::vector<double>> k; // k[][0..3]
    std::vector<std::vector<double>> r; // r[][0..3]
    std::vector<std::vector<double>> q; // q[][0..3]
    std::vector<std::vector<double>> x; // x[0], x[1], x[2]
    std::array<double,2> rearOmega; 
    std::array<double,2> rearTorque;  

    // differential_equations.hpp で定義された配列をコピー
    std::vector<FunctionPtr> fAllVec;
    std::vector<FunctionPtr> fdAllVec;

    // ここから内部で制御入力を計算する関数群

    // 後輪左右の角速度を計算
    // @return std::array{omega_left, omega_right}
    std::array<double,2> computeRearWheelOmegas(double speed, double steeringAngle);
    std::array<double,2> computeRearWheelTorque(double Fx, double steeringAngle);

    void V1(const std::vector<double>& x_old, int sr_j);
    void V2(const std::vector<double>& x_old, int sr_j);

};

#endif // GET_INPUT_VALUE_HPP
