#ifndef PWM_REPORTER_H
#define PWM_REPORTER_H
#include <ros/ros.h>
#include "../thruster-config/thruster_config.h"
#include <libInterpolate/Interpolate.hpp>

#define PWM_REPORTING_FREQ 10

namespace PWMReporter
{
    class Thruster
    {
        _1D::MonotonicInterpolator<float> &interpolater;

        float clamp;
        float max_thrust;
        float min_thrust;

    public:
        Thruster(_1D::MonotonicInterpolator<float> &interpolater, float min_thrust, float max_thrust);

        // Computes the PWM value for the required thrust (in kgf).
        int compute_pwm(float thrust);
    };
};

#endif