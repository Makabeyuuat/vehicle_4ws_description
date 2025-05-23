#include "dynamics_integrator.hpp"
#include "initial.hpp"
#include "mathFunc.h"		// 数学関数のヘッダーファイル
#include "Bezier.h"			// Bezier 曲線の関数
#include "vehicle.hpp"       // Vehicle クラスの宣言
#include "callback.hpp"     // コールバック関数の宣言
#include "initial.hpp"		// 初期値設定のヘッダーファイル
#include "getInputValue.hpp"
#include "pidTau.hpp"
#include "differential_equations_dynamics.hpp"
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/SVD> 

using namespace std;
DynamicsIntegrator::DynamicsIntegrator(double M_mass,
                                       double I_theta,
                                       double lv,
                                       double g,
                                       double rho,
                                       const PIDGains& drive_gains,
                                       const PIDGains& steer_gains,
                                       double dt)
    : M_mass_(M_mass), I_theta_(I_theta), lv_(lv), g_(g), rho_(rho), dt_(dt),
      drive_pid_(drive_gains, dt), steer_pid_(steer_gains, dt) {}

void DynamicsIntegrator::step(const Eigen::Vector3d& q,
                              const Eigen::Vector3d& qdot,
                              double& phi,
                              double& phidot,
                              double u1,
                              double u2)
{
    // 状態展開
    double x        = q(0);
    double y        = q(1);
    double theta    = q(2);
    double xdot     = qdot(0);
    double ydot     = qdot(1);
    double thetadot = qdot(2);

    // 1) PID制御でτを計算 (式5.11・5.12)
    double u1_act = xdot * cos(theta) + ydot * sin(theta);
    double tau1   = drive_pid_.compute(v1, u1_act);
    Tau1 = tau1;
    double tau2   = steer_pid_.compute(v2, phidot);
    Tau2 = tau2;
    // 2) 一般化駆動力 Qc, Qphi の構築 (式5.15)
    Eigen::Vector3d Qc;
    Qc << tau1 * std::cos(theta),
          tau1 * std::sin(theta),
          0.0;
    double Qphi = tau2;

    // 3) Chapter5 行列の定義 (仮実装: 後から計算式を入れる)
    // 質量行列 Mξ (3x3)
    Eigen::Matrix3d Mxi;
    // TODO: 資料式5.18に基づき要素を設定
    Mxi <<  M_mass_, 0.0, -(lv_ *M_mass_ * sin(theta))/2.0,
            0.0, M_mass_, (lv_ *M_mass_ * cos(theta))/2.0,
            -(lv_ *M_mass_ * sin(theta))/2.0, (lv_ *M_mass_ * cos(theta))/2.0, (2*I_theta_ + M_mass_*((pow(lv_,2)*pow(cos(theta),2))/2.0 + (pow(lv_,2)*pow(sin(theta),2))/2.0))/2.0;

    // コリオリ行列 Cξ (3x3)
    Eigen::Matrix3d Cxi;
    Cxi <<  0.0, 0.0, -(lv_*M_mass_*cos(theta)*thetadot),
            0.0, 0.0, -(lv_*M_mass_*sin(theta)*thetadot),
            0.0, 0.0, 0.0;

    // 重力ベクトル Kξ (3x1)
    Eigen::Vector3d Kxi;
    Kxi << GRAV*M_mass_*sin(rho_), 0.0, -(GRAV*lv_*M_mass_*sin(rho_)*sin(theta))/2.0;

    // 拘束行列 Aξ (3x2)
    Eigen::Matrix<double,3,2> Axi;
    Axi <<  sin(theta + phi), sin(theta),
            -cos(theta + phi), -cos(theta),
            -lv_ * cos(phi), 0.0;

    // ヤコビ行列 J (2x3)
    Eigen::Matrix<double,2,3> J;
    J <<  sin(theta + phi), -cos(theta + phi), -lv_ * cos(phi),
            sin(theta), -cos(theta), 0.0;

    // 拘束付き運動方程式行列 H (5x5)
    Eigen::Matrix<double,5,5> H;
    H.setZero();
    H.block<3,3>(0,0) = Mxi;
    H.block<3,2>(0,3) = Axi;
    H.block<2,3>(3,0) = J;
    // H.block<2,2>(3,3) = Zero

    // 右辺ベクトル b (5x1)
    Eigen::Matrix<double,5,1> b;
    b.setZero();
    b.block<3,1>(0,0) = Qc - Cxi * qdot - Kxi;
    // 非ホロを微分した値dのddq以外の項を代入
    b(3) = -xdot*(thetadot+phidot)*cos(theta+phi) - ydot*(thetadot+phidot)*sin(theta+phi)-lv_*thetadot*phidot*sin(phi);
    b(4) = -xdot*thetadot*cos(theta) + ydot*thetadot*sin(theta);

    // Hの疑似逆行列を計算 
    Eigen::FullPivLU<Eigen::Matrix<double,5,5>> lu(H);
    Eigen::Matrix<double,5,1> sol;
    if (lu.isInvertible()) {
        //通常の solve() で解く
        sol = lu.solve(b);
    }
    else {
        // 特異 ⇒ SVD による擬似逆行列で最小ノルム解を得る
        Eigen::JacobiSVD<Eigen::Matrix<double,5,5>> svd(
            H, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // 特異値ベクトル
        const auto& S = svd.singularValues();
        // トレランスの設定（最大特異値 × 次元 × 機械精度 の目安）
        double tol = S(0) * std::max(H.rows(), H.cols()) * std::numeric_limits<double>::epsilon();
        // 擬似逆行列を構築
        Eigen::VectorXd Sinv(5);
        Sinv.setZero(); 
        for (int i = 0; i < S.size(); ++i) {
            if (S(i) > tol) {
                Sinv(i) = 1.0 / S(i);
            }
            // tol 以下の特異値は 0 にして、ノイズを切り捨て
        }
        // H⁺ = V · S⁺ · Uᵀ
        Eigen::Matrix<double,5,5> H_pinv =svd.matrixV() * Sinv.asDiagonal() * svd.matrixU().transpose();
        // 最小ノルム解
        sol = H_pinv * b;
    }
    
    Eigen::Vector3d qdd = sol.block<3,1>(0,0);
    Eigen::Vector2d lambda = sol.block<2,1>(3,0); 
    double phidd = Qphi;

    //積分用の配列に代入
    x_dd[0] = 0.0;
    x_dd[1] = qdd(0);
    x_dd[2] = qdd(1);
    x_dd[3] = qdd(2);
    x_dd[4] = phidd;
}