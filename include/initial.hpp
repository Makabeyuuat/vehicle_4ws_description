#ifndef INITIAL_HPP
#define INITIAL_HPP
#include <cmath>  // sqrt関数を使うために必要
#include <vector>
#include <Eigen/Dense>



#define PAI 3.14159265358979323846
#define PSdist 300
#define GRAV 9.8
inline constexpr int BEZIER_ORDER = 3; 
inline constexpr int Dim = 4;
inline constexpr int  Q = 100001;  //曲線の分割数PAI
inline constexpr double RAD2DEG = 180.0 / PAI;
inline constexpr double DEG2RAD = 180.0 / PAI;
//コールバックのフラグ変数
inline bool got_body_pose = false;
inline bool got_steering_pose = false;
//３次のベジェ曲線
inline double Bx[BEZIER_ORDER + 1] = { -6.5, 3.0, -3.0, 6.5 };
inline double By[BEZIER_ORDER + 1] = { -1.0, -1.0, 1.0, 1.0 };



//探索用の構造体
typedef struct {
	double d;
	double Cs;
	double Cs1;
	double Cs2;
	int j;
	double Psx;
	double Psy;
	double thp;

}Search;



inline Search sr;




//callback時に入れる変数
inline double true_body_pos[2] = {};
inline double omega_rear[2] = {};
inline double true_body_yaw = 0.0;
inline double true_steering = 0.0;
inline std::vector<double> x_old = std::vector<double>(Dim + 1, 0.0);
inline std::vector<double> x_new = std::vector<double>(Dim + 1, 0.0);
inline std::vector<double> x_input = std::vector<double>(Dim + 1, 0.0);
//曲率の配列を保存
inline double cs[Q][3] = {};
inline double R[Q][2] = {};
inline double dRdq[Q][2] = {};
inline double d2Rdq2[Q][2] = {};
inline double d3Rdq3[Q][2] = {};
inline double d4Rdq4[Q][2] = {};
inline double qs[Q] = {};

//刻み幅と時間
inline double	h, t_max;

//input
//制御入力
inline double  w1;
//inline double a0 = 0.2;
inline double u1, u2;
inline double v1, v2;
inline double thetaT = 0.0;


//フィードバック関数
inline double k1 = 2.0;
inline double k2 = 2.0;



//制御入力の係数
inline double z21, z22;
inline double alpha21, alpha22;




//重心の目標相対位置関数
inline double d0d, dd0d, ddd0d;
inline double theta1d, dtheta1d, ddtheta1d;

//各リンクの長さ
inline double lv = 1.0;

//動力学用変数
inline double vx = 0.0;
inline double vx = 0.0;
inline double dynamic_v = 0.0;
inline double yaw_rate = 0.0;
inline double M_mass = 0.0;
inline double I_theta = 0.0; 
inline double rho = 10.0 * DEG2RAD;
inline double phidot = 0.0;
inline double qdot_twist[3] = {0.0};
inline std::vector<double> x_d = std::vector<double>(Dim + 1, 0.0);
inline std::vector<double> x_dd = std::vector<double>(Dim + 1, 0.0);
Eigen::Map<Eigen::Vector3d> q_map{x_old + 1};
Eigen::Map<Eigen::Vector3d> qdot_map(qdot_twist);
inline dymanic_v = 0.0;



// 初期値設定関数
// 引数: t, dt, x0, x_new
void initial(double &t, double &dt);



#endif // INITIAL_HPP
