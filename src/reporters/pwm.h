#ifndef PWM_REPORTER_H
#define PWM_REPORTER_H

#include "../thruster-config/thruster_config.h"
#include <libInterpolate/Interpolate.hpp>
#include <algorithm>
#include <cmath>

#define PWM_REPORTING_FREQ 10

namespace PWMReporter
{
    class Thruster
    {
    private:
        _1D::MonotonicInterpolator<float> &interpolater;

        float clamp;
        float max_thrust;
        float min_thrust;

    public:
        Thruster(_1D::MonotonicInterpolator<float> &interpolater,
                 float min_thrust,
                 float max_thrust);

        // Computes the PWM value for the required thrust
        int compute_pwm(float thrust);
    };
}

#endif
