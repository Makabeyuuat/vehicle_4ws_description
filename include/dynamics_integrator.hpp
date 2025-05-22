#pragma once

#include <Eigen/Dense>
#include "pid_torque.hpp"

/**
 * @brief DynamicsIntegrator: 制約付き動力学＋操舵ダイナミクス統合クラス
 */
class DynamicsIntegrator {
public:
    DynamicsIntegrator(double M_mass,
                       double I_theta,
                       double l,
                       double g,
                       double rho,
                       const PIDGains& drive_gains,
                       const PIDGains& steer_gains,
                       double dt);

    /**
     * @brief 1ステップの動力学計算 & 数値積分
     * @param q      [x, y, θ]
     * @param qdot   [ẋ, ẏ, θ̇]
     * @param phi    操舵角 φ
     * @param phi_dot 操舵角速度 φ̇
     * @param u1     目標前進速度 u1
     * @param u2     目標操舵速度 u2
     */
    void step(Eigen::Vector3d& q,
              Eigen::Vector3d& qdot,
              double& phi,
              double& phi_dot,
              double u1,
              double u2);

private:
    double M_mass_, I_theta_, l_, g_, rho_, dt_;
    PIDTorque drive_pid_;
    PIDTorque steer_pid_;
};