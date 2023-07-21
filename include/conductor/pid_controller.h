#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ros/ros.h>

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double outputMin, double outputMax);
    double compute(double setpoint, double input, double dt);

private:
    double m_kp;
    double m_ki;
    double m_kd;
    double m_outputMin;
    double m_outputMax;
    double m_error;
    double m_lastError;
    double m_integral;
    double m_derivative;

    double m_sample_time;
    ros::Time m_last_time;
    ros::Time m_current_time;
};

#endif // PID_CONTROLLER_H