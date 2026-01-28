#include "thrust.h"
#include "../thruster-config/thruster_config.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <pthread.h>
#include <unistd.h>
#include <vector>
#include <algorithm>

// Globals (kept same structure)
static rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub;
static std_msgs::msg::Float32MultiArray::SharedPtr msg;
static ThrusterConfig config;
static float *thrust_vector;
static pthread_t thread;
static rclcpp::Node::SharedPtr node_ptr;

void *ThrustReporterThread(void *arg)
{
    (void)arg;

    while (rclcpp::ok())
    {
        ThrustReporter::refresh();
        rclcpp::spin_some(node_ptr);
        usleep(THRUST_REPORT_RATE_US);
    }
    return nullptr;
}

void ThrustReporter::init(rclcpp::Node::SharedPtr node)
{
    node_ptr = node;

    // load thruster config
    config = loadThrusterConfig();

    // ensure thrust_vector is non empty
    thrust_vector =
        static_cast<float *>(malloc(sizeof(float) *
                                     config.spec.number_of_thrusters));

    for (int i = 0; i < config.spec.number_of_thrusters; i++)
        thrust_vector[i] = 0.0f;

    // create publisher and message
    pub = node_ptr->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/pecka_tvmc/control/thrust", 50);

    msg = std::make_shared<std_msgs::msg::Float32MultiArray>();
    msg->data.resize(config.spec.number_of_thrusters);

    RCLCPP_INFO(node_ptr->get_logger(),
                "Will start publishing thrust values to %s.",
                pub->get_topic_name());

    // start reporter thread
    pthread_create(&thread, nullptr, ThrustReporterThread, nullptr);
}

void ThrustReporter::refresh()
{
    for (int i = 0; i < config.spec.number_of_thrusters; i++)
        msg->data[i] = thrust_vector[i];

    pub->publish(*msg);
}

void ThrustReporter::report(float *tvec)
{
    std::copy(tvec,
              tvec + config.spec.number_of_thrusters,
              thrust_vector);
}

void ThrustReporter::kill()
{
    // stop thread
    pthread_cancel(thread);
    pthread_join(thread, nullptr);

    pub.reset();
    msg.reset();

    free(thrust_vector);
    thrust_vector = nullptr;
}

// backwards-compatibility

void ThrustReporter::writeThrusterValues(float *thrust_vector)
{
    ThrustReporter::report(thrust_vector);
}

void ThrustReporter::shutdown()
{
    ThrustReporter::kill();
}
