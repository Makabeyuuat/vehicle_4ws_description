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

    //PID制御でtauを計算
    double u1_act = xdot * cos(theta) + ydot * sin(theta);
    double tau1   = drive_pid_.compute(u1, u1_act);
    Tau1 = tau1;
    double tau2   = steer_pid_.compute(u2, phidot);
    Tau2 = tau2;
    //駆動力
    Eigen::Vector3d Qc;
    Qc << tau1 * std::cos(theta),
          tau1 * std::sin(theta),
          0.0;
    double Qphi = tau2;

    //各行列を定義
    // 質量行列(3x3)
    Eigen::Matrix3d Mxi;
    Mxi <<  M_mass_, 0.0, -(lv_ *M_mass_ * sin(theta))/2.0,
            0.0, M_mass_, (lv_ *M_mass_ * cos(theta))/2.0,
            -(lv_ *M_mass_ * sin(theta))/2.0, (lv_ *M_mass_ * cos(theta))/2.0, (2*I_theta_ + M_mass_*((pow(lv_,2)*pow(cos(theta),2))/2.0 + (pow(lv_,2)*pow(sin(theta),2))/2.0))/2.0;

    // コリオリ行列(3x3)
    Eigen::Matrix3d Cxi;
    Cxi <<  0.0, 0.0, -(lv_*M_mass_*cos(theta)*thetadot),
            0.0, 0.0, -(lv_*M_mass_*sin(theta)*thetadot),
            0.0, 0.0, 0.0;

    // 重力ベクトル(3x1)
    Eigen::Vector3d Kxi;
    Kxi << GRAV*M_mass_*sin(rho_), 0.0, -(GRAV*lv_*M_mass_*sin(rho_)*sin(theta))/2.0;

    // 拘束行列(3x2)
    Eigen::Matrix<double,3,2> Axi;
    Axi <<  sin(theta + phi), sin(theta),
            -cos(theta + phi), -cos(theta),
            -lv_ * cos(phi), 0.0;

    // ヤコビ行列(2x3)
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
    Eigen::MatrixXd Hxi_pinv = (H.transpose() * H).inverse()* H.transpose();
    Eigen::VectorXd sol= Hxi_pinv * b;
    
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