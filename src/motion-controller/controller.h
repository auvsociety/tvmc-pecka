#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <map>

#include "../PID-Controller/PID_controller.h"
#include "../reporters/thrust.h"
#include "../thruster-config/thruster_config.h"

// ROS 2 messages
#include "pecka_tvmc_msg/msg/command.hpp"
#include "pecka_tvmc_msg/msg/control_mode.hpp"
#include "pecka_tvmc_msg/msg/current_point.hpp"
#include "pecka_tvmc_msg/msg/do_f.hpp"
#include "pecka_tvmc_msg/msg/pid_constants.hpp"
#include "pecka_tvmc_msg/msg/pid_limits.hpp"
#include "pecka_tvmc_msg/msg/target_point.hpp"
#include "pecka_tvmc_msg/msg/thrust.hpp"

#define CLOSED_LOOP_MODE 0
#define OPEN_LOOP_MODE 1

namespace msg = pecka_tvmc_msg::msg;

class MotionController : public rclcpp::Node
{
public:
    explicit MotionController();
    ~MotionController();

    bool online{true};

    // Control-mode management
    void setControlMode(uint8_t dof, bool mode);

    // PID tuning
    void setPIDConstants(uint8_t dof, float kp, float ki, float kd,
                         float acceptable_error, float ko = 0);

    void setPIDLimits(uint8_t dof, float output_min, float output_max,
                      float integral_min, float integral_max);

    // Closed-loop control
    void setTargetPoint(uint8_t dof, float target);
    void updateCurrentPoint(uint8_t dof, float current);

    // Open-loop control
    void setThrust(uint8_t dof, float thrust);

    // Utility
    void resetAllThrusters();
    void refresh();
    void updateThrustValues();

private:
    float limitToRange(float value, float minimum, float maximum);

private:
    // Thrust per degree of freedom
    float thrust_[6]{};

    // Control mode per DoF
    bool control_modes_[6]{};

    // PID controllers
    PIDController controllers_[6];

    // Thruster configuration
    ThrusterConfig config_;

    // Thruster maps per DoF
    std::map<uint8_t, std::vector<float>> thruster_map_;

    // Final thrust vector
    std::vector<float> thrust_vector_;

    // ROS 2 subscriptions
    rclcpp::Subscription<msg::Command>::SharedPtr sub_command_;
    rclcpp::Subscription<msg::ControlMode>::SharedPtr sub_control_mode_;
    rclcpp::Subscription<msg::CurrentPoint>::SharedPtr sub_current_point_;
    rclcpp::Subscription<msg::PidConstants>::SharedPtr sub_pid_constants_;
    rclcpp::Subscription<msg::PidLimits>::SharedPtr sub_pid_limits_;
    rclcpp::Subscription<msg::TargetPoint>::SharedPtr sub_target_point_;
    rclcpp::Subscription<msg::Thrust>::SharedPtr sub_thrust_;
};

#endif
