#pragma once

/**
 * @brief PIDTorque: PDF 式 (5.11)–(5.14) に忠実な PID 制御器
 */
struct PIDGains {
    double Kp;   ///< 比例ゲイン
    double Ki;   ///< 積分ゲイン
    double Kd;   ///< 微分ゲイン
};

class PIDTorque {
public:
    PIDTorque(const PIDGains& gains, double dt)
      : gains_(gains)
      , dt_(dt)
      , prev_error_(0.0)
      , integral_(0.0)
      , first_step_(true)
    {}

    double compute(double u, double u_act) {
        //誤差
        double e = u - u_act;

        // 2) 積分 (Eq.5.13)
        if (first_step_) {
            integral_ = gains_.Ki * e;  
        } else {
            integral_ += gains_.Ki * e * dt_;
        }

        // 3) 微分 (Eq.5.14)
        double derivative;
        if (first_step_) {
            derivative  = gains_.Kd * e;
            first_step_ = false;
        } else {
            derivative  = gains_.Kd * (e - prev_error_) / dt_;
        }
        prev_error_ = e;

        // 4) 合成 (Eq.5.11/5.12)
        return gains_.Kp * e
             + integral_
             + derivative;
    }

private:
    PIDGains gains_;
    double   dt_;
    double   prev_error_;
    double   integral_;
    bool     first_step_;
};
