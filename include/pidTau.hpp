#pragma once

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

        //積分 
        if (first_step_) {
            integral_ = gains_.Ki * e;  
        } else {
            integral_ += gains_.Ki * e * dt_;
        }

        //微分
        double derivative;
        if (first_step_) {
            derivative  = gains_.Kd * e;
            first_step_ = false;
        } else {
            derivative  = gains_.Kd * (e - prev_error_) / dt_;
        }
        prev_error_ = e;

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
