#pragma once

#include <Eigen/Dense>

/**
@brief PIDTorque: 速度差分に対するPID制御器
 */
struct PIDGains {
    double Kp;
    double Ki;
    double Kd;
};

class PIDTorque {
public:
    PIDTorque(const PIDGains& gains, double dt)
        : gains_(gains), dt_(dt), prev_error_(0.0), integral_(0.0) {}

    /**@param u      目標入力
     @param u_act  実効入力
     @return       計算されたトルク τ
     */
    double compute(double u, double u_act) {
        double error = u - u_act;
        integral_ += error * dt_;
        double derivative = (error - prev_error_) / dt_;
        prev_error_ = error;
        return gains_.Kp * error + gains_.Ki * integral_ + gains_.Kd * derivative;
    }

private:
    PIDGains gains_;
    double dt_;
    double prev_error_;
    double integral_;
};