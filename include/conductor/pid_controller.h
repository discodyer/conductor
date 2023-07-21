#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ros/time.h>
#include <ros/duration.h>

class PIDController {
public:
    PIDController(double kp, double ki, double kd,double windup_guard, double output_bound, double sample_time = 0.1);
    double calculateOutput(double feedback_value);
    double getBoundedOutput(double feedback_value);
    void clear();

    void setKp(double proportional_gain);
    void setKi(double integral_gain);
    void setKd(double derivative_gain);
    void setWindup(double windup);
    void setSampleTime(double sample_time);
    void setOutputBound(double output_bound);

private:
    double kp_;
    double ki_;
    double kd_;
    double output_bound_;
    double error_;
    double last_error_;
    double integral_;
    double derivative_;
    double windup_guard_;
    double setpoint_;
    double p_term_;
    double i_term_;
    double d_term_;
    double last_control_output_;

    double sample_time_;
    ros::Time last_time_;
    ros::Time current_time_;
};

#endif // PID_CONTROLLER_H