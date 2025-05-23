#pragma once
#include <algorithm>
#include <Eigen/Dense>

/**
 * @brief PIDTorque: 速度差分に対するPID制御器（アンチワインドアップ＋微分制限付き）
 */
struct PIDGains {
    double Kp;  // 比例ゲイン
    double Ki;  // 積分ゲイン
    double Kd;  // 微分ゲイン
    double i_max; // 積分飽和上限
    double d_max; // 微分飽和上限
};

class PIDTorque {
public:
    PIDTorque(const PIDGains& gains, double dt)
      : gains_(gains), dt_(dt), prev_error_(0.0), integral_(0.0)
    {}

    /**
     * @param u     目標入力（u1 または u2）
     * @param u_act 実効入力（u1_act または phi_dot）
     * @return      出力トルク τ
     */
    double compute(double u, double u_act) {
        // 1) 誤差
        double error = u - u_act;

        // 2) 積分項 (アンチワインドアップ)
        integral_ += error * dt_;
        integral_ = std::clamp(integral_, -gains_.i_max, gains_.i_max);

        // 3) 微分項 (ノイズ抑制)
        double raw_deriv = (error - prev_error_) / dt_;
        double derivative = std::clamp(raw_deriv, -gains_.d_max, gains_.d_max);

        // 4) 更新
        prev_error_ = error;

        // 5) PID 合成
        return gains_.Kp * error
             + gains_.Ki * integral_
             + gains_.Kd * derivative;
    }

private:
    PIDGains gains_;
    double dt_;
    double prev_error_;
    double integral_;
};
