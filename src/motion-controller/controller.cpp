#include <rclcpp/rclcpp.hpp>
#include "controller.h"

class MotionControllerNode : public rclcpp::Node
{
public:
    MotionControllerNode()
        : Node("pecka_tvmc")
    {
        RCLCPP_INFO(get_logger(), "Starting TVMC on %s",
                    get_namespace());

        // Load thruster configuration
        config = loadThrusterConfig();

        // Initialize controllers
        for (int d = 0; d < 6; d++)
        {
            control_modes[d] = OPEN_LOOP_MODE;
            controllers[d].setConstants(1, 1, 1, 0.001);
            controllers[d].setMinMaxLimits(
                config.spec.min_thrust,
                config.spec.max_thrust,
                config.spec.min_thrust / 2,
                config.spec.max_thrust / 2);

            thrust[d] = 0;
        }

        // Angular controllers
        controllers[msg::DoF::YAW].setAngular(true);
        controllers[msg::DoF::PITCH].setAngular(true);
        controllers[msg::DoF::ROLL].setAngular(true);

        // Thruster maps
        thruster_map[msg::DoF::SURGE] = config.vectors.surge;
        thruster_map[msg::DoF::SWAY]  = config.vectors.sway;
        thruster_map[msg::DoF::HEAVE] = config.vectors.heave;
        thruster_map[msg::DoF::YAW]   = config.vectors.yaw;
        thruster_map[msg::DoF::PITCH] = config.vectors.pitch;
        thruster_map[msg::DoF::ROLL]  = config.vectors.roll;

        // Initialize thrust vector
        thrust_vector.assign(config.spec.number_of_thrusters, 0.0f);

        auto qos = rclcpp::QoS(50);

        // Subscriptions
        sub_command = create_subscription<msg::Command>(
            "/pecka_tvmc/control/command", qos,
            [this](msg::Command::SharedPtr x)
            {
                if (x->command == x->REFRESH)
                    refresh();
                else if (x->command == x->RESET_THRUSTERS)
                    resetAllThrusters();
                else if (x->command == x->SHUT_DOWN)
                    online = false;
            });

        sub_control_mode = create_subscription<msg::ControlMode>(
            "/pecka_tvmc/control/control_mode", qos,
            [this](msg::ControlMode::SharedPtr x)
            {
                setControlMode(x->dof, x->mode);
            });

        sub_current_point = create_subscription<msg::CurrentPoint>(
            "/pecka_tvmc/control/current_point", qos,
            [this](msg::CurrentPoint::SharedPtr x)
            {
                updateCurrentPoint(x->dof, x->current);
            });

        sub_pid_constants = create_subscription<msg::PidConstants>(
            "/pecka_tvmc/control/pid_constants", qos,
            [this](msg::PidConstants::SharedPtr x)
            {
                setPIDConstants(
                    x->dof, x->kp, x->ki, x->kd,
                    x->acceptable_error, x->ko);
            });

        sub_pid_limits = create_subscription<msg::PidLimits>(
            "/pecka_tvmc/control/pid_limits", qos,
            [this](msg::PidLimits::SharedPtr x)
            {
                setPIDLimits(
                    x->dof, x->output_min, x->output_max,
                    x->integral_min, x->integral_max);
            });

        sub_target_point = create_subscription<msg::TargetPoint>(
            "/pecka_tvmc/control/target_point", qos,
            [this](msg::TargetPoint::SharedPtr x)
            {
                setTargetPoint(x->dof, x->target);
            });

        sub_thrust = create_subscription<msg::Thrust>(
            "/pecka_tvmc/control/thrust", qos,
            [this](msg::Thrust::SharedPtr x)
            {
                setThrust(x->dof, x->thrust);
            });
    }

    ~MotionControllerNode()
    {
        ThrustReporter::shutdown();
        RCLCPP_INFO(get_logger(), "Shutting down TVMC");
    }

    bool online{true};

    void init()
    {
        ThrustReporter::init(shared_from_this());
        RCLCPP_INFO(get_logger(), "ThrustReporter initialized.");
    }

private:
    void setControlMode(uint8_t dof, bool mode)
    {
        control_modes[dof] = mode;

        if (mode == CLOSED_LOOP_MODE)
            controllers[dof].reset();
        else
            setThrust(dof, 0);
    }

    void setPIDConstants(uint8_t dof, float kp, float ki, float kd,
                         float acceptable_error, float ko)
    {
        controllers[dof].setConstants(kp, ki, kd, acceptable_error, ko);
    }

    void setPIDLimits(uint8_t dof, float out_min, float out_max,
                      float int_min, float int_max)
    {
        controllers[dof].setMinMaxLimits(out_min, out_max, int_min, int_max);
    }

    void setTargetPoint(uint8_t dof, float target)
    {
        controllers[dof].setTargetValue(target);

        if (control_modes[dof] == OPEN_LOOP_MODE)
            return;

        thrust[dof] = controllers[dof].updateOutput();
        updateThrustValues();
    }

    void updateCurrentPoint(uint8_t dof, float current)
    {
        controllers[dof].setCurrentValue(current);

        if (control_modes[dof] == OPEN_LOOP_MODE)
            return;

        thrust[dof] = controllers[dof].updateOutput();
        updateThrustValues();
    }

    void setThrust(uint8_t dof, float tx)
    {
        if (control_modes[dof] == CLOSED_LOOP_MODE)
        {
            RCLCPP_ERROR(get_logger(),
                         "[DOF %d] Closed-loop enabled, cannot set thrust manually",
                         dof);
            return;
        }

        thrust[dof] = tx;
        updateThrustValues();
    }

    void resetAllThrusters()
    {
        for (int d = 0; d < 6; d++)
            thrust[d] = 0;
    }

    void refresh()
    {
        ThrustReporter::refresh();
    }

    void updateThrustValues()
    {
        for (int i = 0; i < config.spec.number_of_thrusters; i++)
        {
            thrust_vector[i] = 0;

            for (int dof = 0; dof < 6; dof++)
                thrust_vector[i] += thrust[dof] * thruster_map[dof][i];

            thrust_vector[i] = limitToRange(
                thrust_vector[i],
                config.spec.min_thrust,
                config.spec.max_thrust);
        }

        ThrustReporter::writeThrusterValues(thrust_vector.data());
    }

    float limitToRange(float value, float min, float max)
    {
        return std::min(std::max(value, min), max);
    }

private:
    ThrusterConfig config;

    bool control_modes[6];
    float thrust[6];

    PIDController controllers[6];
    std::map<uint8_t, std::vector<float>> thruster_map;
    std::vector<float> thrust_vector;

    rclcpp::Subscription<msg::Command>::SharedPtr sub_command;
    rclcpp::Subscription<msg::ControlMode>::SharedPtr sub_control_mode;
    rclcpp::Subscription<msg::CurrentPoint>::SharedPtr sub_current_point;
    rclcpp::Subscription<msg::PidConstants>::SharedPtr sub_pid_constants;
    rclcpp::Subscription<msg::PidLimits>::SharedPtr sub_pid_limits;
    rclcpp::Subscription<msg::TargetPoint>::SharedPtr sub_target_point;
    rclcpp::Subscription<msg::Thrust>::SharedPtr sub_thrust;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionControllerNode>();
    node->init();  // Initialize after construction so shared_from_this() works
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
