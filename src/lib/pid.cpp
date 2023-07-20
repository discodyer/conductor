#include "conductor/pid_controller.hpp"

PIDController::PIDController(double kp, double ki, double kd, double outputMin, double outputMax)
    : m_kp(kp), m_ki(ki), m_kd(kd), m_outputMin(outputMin), m_outputMax(outputMax),
      m_error(0.0), m_lastError(0.0), m_integral(0.0), m_derivative(0.0)
{
}

double PIDController::compute(double setpoint, double input, double dt)
{
    // 计算误差
    m_error = setpoint - input;

    // 计算积分项
    m_integral += m_error * dt;

    // 计算微分项
    m_derivative = (m_error - m_lastError) / dt;

    // 计算输出
    double output = m_kp * m_error + m_ki * m_integral + m_kd * m_derivative;

    // 限制输出范围
    if (output < m_outputMin)
    {
        output = m_outputMin;
    }
    else if (output > m_outputMax)
    {
        output = m_outputMax;
    }

    // 保存上一次的误差
    m_lastError = m_error;

    return output;
}
