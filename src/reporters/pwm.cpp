#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "pwm.h"
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>

using namespace PWMReporter;

class PWMNode : public rclcpp::Node
{
public:
    PWMNode()
        : Node("pecka_pvmc")
    {
        RCLCPP_INFO(get_logger(), "Loading Thruster configuration...");

        config_ = loadThrusterConfig();

        // Create interpolators
        for (auto &map : config_.thrust_maps)
        {
            _1D::MonotonicInterpolator<float> interp;
            interp.setData(map.second.thrust, map.second.pwm);
            interpolaters_[map.first] = interp;
        }

        // Create thrusters
        for (auto &tx : config_.spec.thruster_types)
        {
            if (interpolaters_.find(tx) == interpolaters_.end())
            {
                RCLCPP_FATAL(get_logger(),
                             "Unable to find thrust map for thruster type %s",
                             tx.c_str());
                rclcpp::shutdown();
                return;
            }

            auto &interp = interpolaters_[tx];
            float min = *std::min_element(
                config_.thrust_maps[tx].thrust.begin(),
                config_.thrust_maps[tx].thrust.end());

            float max = *std::max_element(
                config_.thrust_maps[tx].thrust.begin(),
                config_.thrust_maps[tx].thrust.end());

            thrusters_.emplace_back(interp, min, max);
        }

        thrust_vector_.assign(config_.spec.number_of_thrusters, 0.0f);

        pwm_msg_.data.resize(config_.spec.number_of_thrusters);

        pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            "/control/pwm", 50);

        sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pecka_tvmc/control/thrust", 10,
            std::bind(&PWMNode::thrustCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "PWM Reporter node started.");
    }

    ~PWMNode()
    {
        // Zero thrusters on shutdown
        for (size_t i = 0; i < thrusters_.size(); i++)
            pwm_msg_.data[i] = thrusters_[i].compute_pwm(0);

        pub_->publish(pwm_msg_);
        RCLCPP_INFO(get_logger(), "Thrusters zeroed. Shutting down.");
    }

private:
    void thrustCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        bool change = false;

        for (size_t i = 0; i < thrust_vector_.size(); i++)
        {
            if (msg->data[i] != thrust_vector_[i])
            {
                change = true;
                break;
            }
        }

        if (!change)
            return;

        thrust_vector_ = msg->data;

        for (size_t i = 0; i < thrusters_.size(); i++)
            pwm_msg_.data[i] = thrusters_[i].compute_pwm(thrust_vector_[i]);

        pub_->publish(pwm_msg_);
    }

private:
    ThrusterConfig config_;

    std::map<std::string, _1D::MonotonicInterpolator<float>> interpolaters_;
    std::vector<Thruster> thrusters_;

    std::vector<float> thrust_vector_;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;

    std_msgs::msg::Int32MultiArray pwm_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PWMNode>());
    rclcpp::shutdown();
    return 0;
}
