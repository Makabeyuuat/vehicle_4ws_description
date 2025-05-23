#include "getInputValue_dynamics.hpp"
#include "initial.hpp"
#include "differential_equations_dynamics.hpp"
#include "mathFunc.h"

using namespace std;

// コンストラクタ
getInputValue::getInputValue(double h)
    : h(h),
    k(Dim+1, std::vector<double>(4,0.0)),
    r(Dim+1, std::vector<double>(4,0.0)),
    q(Dim+1, std::vector<double>(4,0.0)),
    x_stage(3,   std::vector<double>(Dim+1,0.0)),
    rearOmega{0.0, 0.0},
    fAllVec(fAll.begin(), fAll.end()),
    fdAllVec(fdAll.begin(), fdAll.end())
   
{

}

array<double,2> getInputValue::computeRearWheelOmegas(double speed, double steeringAngle) {
    const double wheelRadius = 0.3;  // ホイール半径[m]
    const double L = lv;            // ホイールベース[m]
    const double W = 0.2;            // トレッド幅[m]
    array<double,2> omegas;

    if (fabs(steeringAngle) < 1e-6) {
        double omega = speed / wheelRadius;
        omegas[0] = omega;
        omegas[1] = omega;
        return omegas;
    }
    double absPhi = fabs(steeringAngle);
    double R = L / tan(absPhi);
    double R_in  = R - W/2.0;
    double R_out = R + W/2.0;
    double v_in  = speed * (R_in  / R);
    double v_out = speed * (R_out / R);
    double omega_in  = v_in  / wheelRadius;
    double omega_out = v_out / wheelRadius;

    if (steeringAngle > 0) {
        // 左折: 左が内輪
        omegas[0] = omega_in;
        omegas[1] = omega_out;
    } else {
        // 右折: 右が内輪
        omegas[0] = omega_out;
        omegas[1] = omega_in;
    }
    return omegas;
}


void getInputValue::rungeKutta(std::vector<double>& x_old, std::vector<double>& x_d) {
    int n = static_cast<int>(x_old.size());  // n = Dim+1
    //qdを積分
    // 第1段階
        for (int i = 0; i < n; i++) {
            double dx = x_d[i];
            k[i][0] = h * dx;
            r[i][0] = (k[i][0] - 2.0 * q[i][3]) / 2.0;
            x_stage[0][i] = x_old[i] + r[i][0];
            q[i][0] = q[i][3] + 3.0 * r[i][0] - k[i][0] / 2.0;
        }   
        // 第2段階
        for (int i = 0; i < n; i++) {
            double dx = fAllVec[i](x_stage[0]);
            k[i][1] = h * dx;
            r[i][1] = (1.0 - sqrt(0.5)) * (k[i][1] - q[i][0]);
            x_stage[1][i] = x_stage[0][i] + r[i][1];
            q[i][1] = q[i][0] + 3.0 * r[i][1] - (1.0 - sqrt(0.5)) * k[i][1];
        }
        // 第3段階
        for (int i = 0; i < n; i++) {
            double dx = fAllVec[i](x_stage[0]);
            k[i][2] = h * dx;
            r[i][2] = (1.0 + sqrt(0.5)) * (k[i][2] - q[i][1]);
            x_stage[2][i] = x_stage[1][i] + r[i][2];
            q[i][2] = q[i][1] + 3.0 * r[i][2] - (1.0 + sqrt(0.5)) * k[i][2];
        }
        // 第4段階
        for (int i = 0; i < n; i++) {
            double dx = fAllVec[i](x_stage[0]);
            k[i][3] = h * dx;
            r[i][3] = (k[i][3] - 2.0 * q[i][2]) / 6.0;
            x_new[i] = x_stage[2][i] + r[i][3];
            q[i][3] = q[i][2] + 3.0 * r[i][3] - k[i][3] / 2.0;
        }
        for(int i=1;i<n; i++){
            x_old[i] = x_new[i];
        }
        x_old[0] = x_old[0] + h;  // 時間の更新

        dynamic_v = x_d[1]*cos(x_old[3]) + x_d[2]*sin(x_old[3]);

        // 後輪左右の角速度をクラスメンバに格納
        rearOmega = computeRearWheelOmegas(dynamic_v, x_old[4]);
        omega_rear[0] = rearOmega[0];  // 左後輪
        omega_rear[1] = rearOmega[1];  // 右後輪
    
}

void getInputValue::ddrungeKutta(std::vector<double>& x_d, std::vector<double>& x_dd) {
    std::vector<double> x_newd = std::vector<double>(Dim +1, 0.0);
    int n = static_cast<int>(x_dd.size()); 
    //qdを積分
    // 第1段階
        for (int i = 0; i < n; i++) {
            double ddx = x_dd[i];
            k[i][0] = h * ddx;
            r[i][0] = (k[i][0] - 2.0 * q[i][3]) / 2.0;
            x_stage[0][i] = x_d[i] + r[i][0];
            q[i][0] = q[i][3] + 3.0 * r[i][0] - k[i][0] / 2.0;
        }   
        // 第2段階
        for (int i = 0; i < n; i++) {
            double ddx = fdAllVec[i](x_stage[0]);
            k[i][1] = h * ddx;
            r[i][1] = (1.0 - sqrt(0.5)) * (k[i][1] - q[i][0]);
            x_stage[1][i] = x_stage[0][i] + r[i][1];
            q[i][1] = q[i][0] + 3.0 * r[i][1] - (1.0 - sqrt(0.5)) * k[i][1];
        }
        // 第3段階
        for (int i = 0; i < n; i++) {
            double ddx = fdAllVec[i](x_stage[0]);
            k[i][2] = h * ddx;
            r[i][2] = (1.0 + sqrt(0.5)) * (k[i][2] - q[i][1]);
            x_stage[2][i] = x_stage[1][i] + r[i][2];
            q[i][2] = q[i][1] + 3.0 * r[i][2] - (1.0 + sqrt(0.5)) * k[i][2];
        }
        // 第4段階
        for (int i = 0; i < n; i++) {
            double ddx = fdAllVec[i](x_stage[0]);
            k[i][3] = h * ddx;
            r[i][3] = (k[i][3] - 2.0 * q[i][2]) / 6.0;
            x_newd[i] = x_stage[2][i] + r[i][3];
            q[i][3] = q[i][2] + 3.0 * r[i][3] - k[i][3] / 2.0;
        }
        // ... 更新結果を x_new に格納
        for(int i=0;i<n; i++){
            x_d[i] = x_newd[i];
        }     
}

void getInputValue::getU(std::vector<double>& x_old, int sr_j) {
    // --- 制御入力の計算 ---
    // 各内部関数を呼び出して制御入力を計算
    thetaT = x_old[3] - atan2(dRdq[sr_j][1], dRdq[sr_j][0]);

    x_old[4] = x_old[4] - thetaT;

    V1(x_old, sr_j);
    V2(x_old, sr_j);


}

void getInputValue::getXInput(std::vector<double>& x_old, std::vector<double>& x_input){
	
}
// getter 関数
// double getInputValue::getU4()  const { return u4; }
// double getInputValue::getU5()  const { return u5; }
// double getInputValue::getU6()  const { return u6; }
// double getInputValue::getU7()  const { return u7; }
// double getInputValue::getU8()  const { return u8; }
// double getInputValue::getU9()  const { return u9; }
// double getInputValue::getU10() const { return u10; }
// double getInputValue::getU11() const { return u11; }
// double getInputValue::getU12() const { return u12; }

// --- 制御入力計算用内部関数 ---
void getInputValue::V1(const std::vector<double>& x_old, int sr_j) {


	v1 = ((1 - sr.d * sr.Cs) / cos(thetaT - atan2(dRdq[sr_j][1], dRdq[sr_j][0]))) * u1;


}


void getInputValue::V2(const std::vector<double>& x_old, int sr_j) {

	double dx2ds = 0.0;
	double dx2dd = 0.0;
	double dx2dthp = 0.0;
	double dx2dphi = 0.0;
	double a1;
	double a2;

	

	//??lv?l?v?Z
	double z1 = -sr.Cs1 * sr.d * tan(thetaT)
		- sr.Cs * (1 - sr.d * sr.Cs) * ((1 + pow(sin(thetaT), 2)) / pow(cos(thetaT), 2))
		+ pow((1 - sr.d * sr.Cs), 2) * tan(x_old[4]) / (lv * pow(cos(thetaT), 3));
	double z2 = ((1 - sr.d * sr.Cs) * tan(thetaT)) / u1;
	double z3 = sr.d / pow(u1, 2);
	double a = -1.5;

	double p1, p2, p3;

	p1 = 3 * a;
	p2 = -3 * a * a;
	p3 = a * a * a;

	u2 = p1 * z1 + p2 * z2 + p3 * z3;


	dx2ds = -sr.Cs2 * sr.d * tan(thetaT)
		- sr.Cs1 * (1 - sr.d * sr.Cs) * ((1 + pow(sin(thetaT), 2)) / pow(cos(thetaT), 2))
		+ sr.d * sr.Cs * sr.Cs1 * ((1 + pow(sin(thetaT), 2))) / pow(cos(thetaT), 2)
		- sr.d * sr.Cs1 * (2 * (1 - sr.d * sr.Cs) * tan(x_old[4])) / (lv * pow(cos(thetaT), 3)) -sr.Cs-sr.d;

	dx2dd = -sr.Cs1 * tan(thetaT)
		+ sr.Cs * sr.Cs * ((1 + pow(sin(thetaT), 2)) / pow(cos(thetaT), 2))
		- (2 * (1 - sr.d * sr.Cs) * tan(x_old[4]) * sr.Cs) / (lv * pow(cos(thetaT), 3));


	dx2dthp = -sr.Cs1 * sr.d / pow(cos(thetaT), 2)
		- sr.Cs * (1 - sr.d * sr.Cs) * 4 * sin(thetaT) / pow(cos(thetaT), 3)
		+ 3 * (pow((1 - sr.d * sr.Cs), 2) * tan(x_old[4]) * sin(thetaT)) / (lv * pow(cos(thetaT), 4));

	dx2dphi = pow((1 - sr.d * sr.Cs), 2) / (lv * pow(cos(thetaT), 3) * pow(cos(x_old[4]), 2));

	a1 = dx2ds + dx2dd * (1 - sr.d * sr.Cs) * tan(thetaT)
		+ dx2dthp * ((tan(x_old[4]) * (1 - sr.d * sr.Cs)) / (lv * cos(thetaT)) - sr.Cs);

	a2 = (lv * pow(cos(thetaT), 3) * pow(cos(x_old[4]), 2)) / pow((1 - sr.d * sr.Cs), 2);



	v2 = a2 * (u2 - a1 * u1);

}

