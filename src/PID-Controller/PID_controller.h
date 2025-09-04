#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <chrono>
#include <thread>

#define PID_ANGULAR_WRAPAROUND 360.0f

class PIDController
{
private:
    float Kp_, Ki_, Kd_, Ko_;
    float current_value_, target_value_;
    float integral_max_, integral_min_;
    float output_max_, output_min_;
    float error_, prev_error_, acceptable_error_;

    std::chrono::high_resolution_clock pid_clock_;
    std::chrono::time_point<std::chrono::high_resolution_clock> prev_time_, current_time_;

    float time_difference_;
    float p_, i_, d_;
    float output_;
    float min_left_, min_right_;
    bool reset_;
    bool angular_;

    float limitToRange(float value, float minimum, float maximum);

public:
    PIDController();
    ~PIDController();

    void setAngular(bool angular);

    /**
     * @brief Sets the constants for the PID controller.
     * 
     * @param Kp The proportional gain.
     * @param Ki The integral gain.
     * @param Kd The derivative gain.
     * @param acceptable_error The acceptable error threshold.
     * @param Ko A fixed offset, if required. (optional, default value is 0).
     */
    void setConstants(float Kp, float Ki, float Kd, float acceptable_error, float Ko = 0);
    
    /**
     * @brief Sets the minimum and maximum limits for the output and integral terms of the PID controller.
     * 
     * @param output_min The minimum value for the output term.
     * @param output_max The maximum value for the output term.
     * @param integral_min The minimum value for the integral term.
     * @param integral_max The maximum value for the integral term.
     */
    void setMinMaxLimits(float output_min, float output_max, float integral_min, float integral_max);
    void setCurrentValue(float current_value);
    void setTargetValue(float target_value);

    float shortestAngularPath(float target, float current);
    float updateOutput();
    float updateOutput(float current_value);
    float updateOutput(float current_value, float target_value);

    void reset();
};

#endif