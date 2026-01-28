#include "PID_controller.h"

#include <limits>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

PIDController::PIDController()
: pid_clock_(RCL_SYSTEM_TIME)
{
    Kp_ = Ki_ = Kd_ = Ko_ = 0;
    p_ = i_ = d_ = 0;

    integral_min_ = output_min_ = std::numeric_limits<float>::min();
    integral_max_ = output_max_ = std::numeric_limits<float>::max();

    prev_time_ = current_time_ = pid_clock_.now();

    error_ = prev_error_ = 0;
    reset_ = true;
}

PIDController::~PIDController()
{
}

void PIDController::setConstants(float Kp, float Ki, float Kd,
                                 float acceptable_error, float Ko)
{
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    acceptable_error_ = acceptable_error;
    Ko_ = Ko;
    reset();
}

void PIDController::setMinMaxLimits(float output_min, float output_max,
                                    float integral_min, float integral_max)
{
    output_min_ = output_min;
    output_max_ = output_max;
    integral_min_ = integral_min;
    integral_max_ = integral_max;
    reset();
}

void PIDController::setCurrentValue(float current_value)
{
    current_value_ = current_value;
}

void PIDController::setTargetValue(float target_value)
{
    target_value_ = target_value;
}

void PIDController::setAngular(bool angular)
{
    angular_ = angular;
}

float PIDController::shortestAngularPath(float target, float current)
{
    float value;
    value = std::fmod(target - current + PID_ANGULAR_WRAPAROUND,
                      PID_ANGULAR_WRAPAROUND);

    if (value > PID_ANGULAR_WRAPAROUND / 2.0f)
        value -= PID_ANGULAR_WRAPAROUND;

    return value;
}

float PIDController::updateOutput()
{
    current_time_ = pid_clock_.now();

    time_difference_ =
        (current_time_ - prev_time_).nanoseconds() / 1e6f;  // ms

    prev_time_ = current_time_;

    // Angular wrap handling
    if (angular_)
        error_ = shortestAngularPath(target_value_, current_value_);
    else
        error_ = target_value_ - current_value_;

    if ((error_ >= 0 && error_ <= acceptable_error_) ||
        (error_ < 0 && error_ >= -acceptable_error_))
    {
        error_ = 0;
    }

    // Proportional
    p_ = Kp_ * error_;

    // Integral
    i_ += Ki_ * error_ * time_difference_;
    i_ = limitToRange(i_, integral_min_, integral_max_);

    // Derivative
    if (reset_)
    {
        d_ = 0;
        reset_ = false;
    }
    else if (error_ != 0)
    {
        d_ = time_difference_
                 ? Kd_ * (error_ - prev_error_) / time_difference_
                 : 0;
        prev_error_ = error_;
    }

    output_ = p_ + i_ + d_ + Ko_;
    output_ = limitToRange(output_, output_min_, output_max_);

    return output_;
}

float PIDController::updateOutput(float current_value)
{
    setCurrentValue(current_value);
    return updateOutput();
}

float PIDController::updateOutput(float current_value, float target_value)
{
    setTargetValue(target_value);
    return updateOutput(current_value);
}

void PIDController::reset()
{
    p_ = i_ = d_ = 0;
    prev_time_ = current_time_ = pid_clock_.now();
    reset_ = true;
}

float PIDController::limitToRange(float value, float minimum, float maximum)
{
    if (value > maximum)
        return maximum;
    else if (value < minimum)
        return minimum;
    else
        return value;
}
