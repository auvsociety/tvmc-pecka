#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H
#include "../PID-Controller/PID_controller.h"
#include "../reporters/thrust.h"
#include "../thruster-config/thruster_config.h"
#include <vector>

#include "rose_tvmc_msg/Command.h"
#include "rose_tvmc_msg/ControlMode.h"
#include "rose_tvmc_msg/CurrentPoint.h"
#include "rose_tvmc_msg/DoF.h"
#include "rose_tvmc_msg/PidConstants.h"
#include "rose_tvmc_msg/PidLimits.h"
#include "rose_tvmc_msg/TargetPoint.h"
#include "rose_tvmc_msg/Thrust.h"

#define CLOSED_LOOP_MODE 0
#define OPEN_LOOP_MODE 1

class MotionController
{
private:
    // Node handle to handle topics
    ros::NodeHandle *nh, *nhc;

    // Thrust in each degree of motion
    float thrust[6];

    // Control modes for each degree of motion
    float control_modes[6];

    // PID controllers for each degree of freedom
    PIDController controllers[6];

    // The MotionController's copy of the Thruster config
    ThrusterConfig config;

    // The thruster map for each degree of freedom
    std::vector<float> thruster_map[6];

    // Final thrust vector for with thrust for each vector
    std::vector<float> thrust_vector;

    // Subscriber for Motion controller commands
    ros::Subscriber sub_command, sub_control_mode, sub_current_point, sub_pid_constants, sub_pid_limits, sub_target_point, sub_thrust;

public:
    bool online = true;

    MotionController(ros::NodeHandle *nh);
    ~MotionController();

    /**
     * Changes control mode for each DoF
     * 
     * @param dof The Degree of Freedom
     * @param mode The Control Mode
    */
    void setControlMode(uint8_t dof, bool mode);

    /**
     * Adjusts PID constants for each DoF
     * 
     * @param dof The Degree of Freedom
     * @param kp Propotional Constant
     * @param ki Integral Constant
     * @param kd Derivative Constant
     * @param acceptable_error Minimum error required for the controller to perform corrections.
    */
    void setPIDConstants(uint8_t dof, float kp, float ki, float kd, float acceptable_error, float ko = 0);

    /**
     * Adjusts PID limits in each DoF
     * 
     * @param dof The Degree of Freedom
     * @param output_min Minimum output thrust from the controller
     * @param output_max Maximum output thrust from the controller
     * @param integral_min Minimum integral contribution to thrust from the controller
     * @param integral_max Maximum integral contribution  thrust from the controller
    */
    void setPIDLimits(uint8_t dof, float output_min, float output_max, float integral_min, float integral_max);

    /**
     * Sets target values in each DoF
     * Works only if closed loop control is enabled
     * 
     * @param dof The Degree of Freedom
     * @param target Target for the PID Controller
    */
    void setTargetPoint(uint8_t dof, float target);
    
    /**
     * Sets current values in each DoF
     * Works only if closed loop control is enabled
     * 
     * @param dof The Degree of Freedom
     * @param target Current value for the PID Controller
    */
    void updateCurrentPoint(uint8_t dof, float current);

    /**
     * Manually sets thrust values in each DoF
     * Works only if open loop control is enabled
     * 
     * @param dof The Degree of Freedom
     * @param thrust Thrust value
    */
    void setThrust(uint8_t dof, float thrust);

    /**
     * Resets all thrusters to zero
    */
    void resetAllThrusters();
    
    /**
     * Requests an immediate refresh fromt the Thrust Reporter
    */
    void refresh();

    /**
     * Updates all thrust values to the Thrust Reporter
    */
    void updateThrustValues();

private:
    /**
     * Simple clamping function
    */
    float limitToRange(float value, float minimum, float maximum);
};

#endif