#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

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
};

#endif // PID_CONTROLLER_HPP