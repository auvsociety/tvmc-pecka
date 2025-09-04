#include <iostream>
#include "controller.h"

namespace msg = rose_tvmc_msg;

MotionController::MotionController(ros::NodeHandle *nhx)
{
    nh = nhx;
    ROS_INFO("Starting TVMC on %s.", nh->getNamespace().c_str());

    // load thruster configuration
    config = loadThrusterConfig();

    // initialize the thrust reporter
    ThrustReporter::init(nh);

    // initialize all controllers,
    // have everything in open loop mode in the beginning.
    // set thrust to 0 for all degrees
    for (int d = 0; d < 6; d++)
    {
        control_modes[d] = OPEN_LOOP_MODE;
        controllers[d].setConstants(1, 1, 1, 0.001);
        controllers[d].setMinMaxLimits(config.spec.min_thrust, config.spec.max_thrust, config.spec.min_thrust / 2, config.spec.max_thrust / 2);
        thrust[d] = 0;
    }

    // set PID controllers to angular mode for angles
    controllers[msg::DoF::YAW].setAngular(true);
    controllers[msg::DoF::PITCH].setAngular(true);
    controllers[msg::DoF::ROLL].setAngular(true);

    // set thruster maps for each degree of freedom
    thruster_map[msg::DoF::SURGE] = config.vectors.surge;
    thruster_map[msg::DoF::SWAY] = config.vectors.sway;
    thruster_map[msg::DoF::HEAVE] = config.vectors.heave;
    thruster_map[msg::DoF::YAW] = config.vectors.yaw;
    thruster_map[msg::DoF::PITCH] = config.vectors.pitch;
    thruster_map[msg::DoF::ROLL] = config.vectors.roll;

    // initialize thrust vector to 0
    for (int i = 0; i < config.spec.number_of_thrusters; i++)
        thrust_vector.push_back(0);

    // setup subcribers
    nhc = new ros::NodeHandle(*nh, "control");

    sub_command = nhc->subscribe<msg::Command>(
        "command", 50,
        [&](const msg::CommandConstPtr &x)
        {
            if (x->Command == x->REFRESH)
                this->refresh();
            if (x->Command == x->RESET_THRUSTERS)
                this->resetAllThrusters();
            if (x->Command == x->SHUT_DOWN)
                this->online = false;
        });

    sub_control_mode = nhc->subscribe<msg::ControlMode>(
        "control_mode", 50, [&](const msg::ControlModeConstPtr &x)
        { this->setControlMode(x->DoF, x->Mode); });

    sub_current_point = nhc->subscribe<msg::CurrentPoint>(
        "current_point", 50, [&](const msg::CurrentPointConstPtr &x)
        { this->updateCurrentPoint(x->DoF, x->Current); });

    sub_pid_constants = nhc->subscribe<msg::PidConstants>(
        "pid_constants", 50, [&](const msg::PidConstantsConstPtr &x)
        { this->setPIDConstants(x->DoF, x->Kp, x->Ki, x->Kd, x->AcceptableError, x->Ko); });

    sub_pid_limits = nhc->subscribe<msg::PidLimits>(
        "pid_limits", 50, [&](const msg::PidLimitsConstPtr &x)
        { this->setPIDLimits(x->DoF, x->OutputMin, x->OutputMax, x->IntegralMin, x->IntegralMax); });

    sub_target_point = nhc->subscribe<msg::TargetPoint>(
        "target_point", 50, [&](const msg::TargetPointConstPtr &x)
        { this->setTargetPoint(x->DoF, x->Target); });

    sub_thrust = nhc->subscribe<msg::Thrust>(
        "thrust", 50, [&](const msg::ThrustConstPtr &x)
        { this->setThrust(x->DoF, x->Thrust); });
}

MotionController::~MotionController()
{
    ThrustReporter::shutdown();
    ROS_INFO("Shutting down TVMC on %s", nh->getNamespace().c_str());
}

void MotionController::setControlMode(uint8_t dof, bool mode)
{
    // set control mode for degree of freedom
    control_modes[dof] = mode;

    // if the mode is set to closed loop mode, reset the PID Controller
    if (mode == CLOSED_LOOP_MODE)
        controllers[dof].reset();
    else
        MotionController::setThrust(dof, 0);
}

void MotionController::setPIDConstants(uint8_t dof, float kp, float ki, float kd, float acceptable_error, float ko)
{
    controllers[dof].setConstants(kp, ki, kd, acceptable_error, ko);
}

void MotionController::setPIDLimits(uint8_t dof, float output_min, float output_max, float integral_min, float integral_max)
{
    controllers[dof].setMinMaxLimits(output_min, output_max, integral_min, integral_max);
}

void MotionController::setTargetPoint(uint8_t dof, float target)
{
    // set target value for controller
    controllers[dof].setTargetValue(target);

    // don't do anything if in open loop mode
    if (control_modes[dof] == OPEN_LOOP_MODE) return;

    // poll output and update thrust
    thrust[dof] = controllers[dof].updateOutput();

    // update thrust values on request
    if (control_modes[dof] == CLOSED_LOOP_MODE)
    MotionController::updateThrustValues();
}

void MotionController::updateCurrentPoint(uint8_t dof, float current)
{
    // update current value for controller
    controllers[dof].setCurrentValue(current);

    // don't do anything if in open loop mode
    if (control_modes[dof] == OPEN_LOOP_MODE) return;

    // poll output and update thrust
    thrust[dof] = controllers[dof].updateOutput();

    // update thrust values on request
    MotionController::updateThrustValues();
}

void MotionController::setThrust(uint8_t dof, float tx)
{
    // ensure control mode is set to open loop for given DoF
    if (control_modes[dof] == CLOSED_LOOP_MODE)
    {
        ROS_ERROR("[DOF %d] %s", dof, "Error, closed loop control enabled, cannot set thrust manually.");
        return;
    }

    // manually set thrust value
    thrust[dof] = tx;

    // update thrust value on request
    MotionController::updateThrustValues();
}

void MotionController::resetAllThrusters()
{
    // set thrust at all degrees of freedom to 0
    // will reset all the thrusters at next update
    for (int d = 0; d < 6; d++)
        thrust[d] = 0;
}

void MotionController::refresh()
{
    // should never really be a need for this unless
    // there is something wrong with the refresh rate
    ThrustReporter::refresh();
}

void MotionController::updateThrustValues()
{
    for (int i = 0; i < config.spec.number_of_thrusters; i++)
    {
        // reset thrust vector to 0
        thrust_vector[i] = 0;

        // add output required by each DoF
        for (int dof = 0; dof < 6; dof++)
            thrust_vector[i] += (thrust[dof] * thruster_map[dof][i]);

        // clamp thrust
        thrust_vector[i] = limitToRange(thrust_vector[i], config.spec.min_thrust, config.spec.max_thrust);
    }

    // report thrust to the thrust reporter
    float *thrust_array = &thrust_vector[0];
    ThrustReporter::writeThrusterValues(thrust_array);
}

float MotionController::limitToRange(float value, float minimum, float maximum)
{
    if (value > maximum)
        return maximum;
    if (value < minimum)
        return minimum;
    return value;
}

int main(int argc, char **argv)
{
    // initialize ros node stuff
    ros::init(argc, argv, "rose_tvmc");
    ros::NodeHandle nh("rose_tvmc");
    ros::Rate rate(1);

    // make a motion controller instance
    auto m = new MotionController(&nh);

    // keep the main loop running as long as ros is okay
    // spin once a second chumma because why not
    while (ros::ok() && m->online)
    {
        rate.sleep();
        ros::spinOnce();
    }

    // shut down the motion controller and thrust reporter
    delete m;

    // good-bye :)
    return 0;
}
