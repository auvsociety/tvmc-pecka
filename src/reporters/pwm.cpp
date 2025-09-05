#include "pwm.h"
#include <cmath>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <signal.h>

using namespace PWMReporter;

ThrusterConfig config = loadThrusterConfig();
std::map<std::string, _1D::MonotonicInterpolator<float>> interpolaters;
std::vector<Thruster> thrusters;
ros::Publisher *pub;
std_msgs::Int32MultiArray *msg;
float *thrust_vector = nullptr;

Thruster::Thruster(_1D::MonotonicInterpolator<float> &interp, float min_thrust, float max_thrust) : interpolater(interp)
{
    this->min_thrust = min_thrust;
    this->max_thrust = max_thrust;
    this->clamp = std::min(std::abs(min_thrust), std::abs(max_thrust));
}

int Thruster::compute_pwm(float thrust)
{
    // clamp = min(max thrust on both sides)
    // thrust = thrust as some unit/spec (usually percentage 0 - 100)
    // full thrust = max of this unit/spec

    // we first find normalized thrust from 0-1, (thrust / full thrust)
    // and the multiply it by the clamp
    // and then restrict this to max or min thrust that can be output by the thruster
    // and then finally interpolate this to PWM 

    return std::round(interpolater(
        std::min(
            std::max(
                clamp * thrust / config.spec.full_thrust,
                this->min_thrust),
            this->max_thrust))) + config.pwm_offset;
}

// Updates the PWM values if there is need to do so
void thrustCallback(const std_msgs::Float32MultiArray::ConstPtr &vec)
{
    // don't report new pwm value no change is observed
    bool change = false;
    for (int i = 0; i < config.spec.number_of_thrusters && !change; i++)
        if (vec->data[i] != thrust_vector[i])
            change = true;
    if (!change)
        return;

    // update current copy
    std::copy(vec->data.begin(), vec->data.end(), thrust_vector);

    // compute pwm vector, and store in msg
    for (int i = 0; i < config.spec.number_of_thrusters; i++)
        msg->data[i] = thrusters[i].compute_pwm(thrust_vector[i]);

    // publish message
    pub->publish(*msg);
}

// Zeroes all thrusters before shutting the node down.
void handle_signint(int sig)
{
    // zero thrusters
    for (int i = 0; i < config.spec.number_of_thrusters; i++)
        msg->data[i] = thrusters[i].compute_pwm(0);
    pub->publish(*msg);

    ROS_INFO("Shutting down PWM Reporter.");

    // showdown node as usual
    ros::shutdown();
}

int main(int argc, char **argv)
{
    // create an interpolater for each thrust map
    for (auto map : config.thrust_maps)
    {
        _1D::MonotonicInterpolator<float> interp;
        interp.setData(map.second.thrust, map.second.pwm);
        interpolaters[map.first] = interp;
    }

    // create thrusters
    for (auto tx : config.spec.thruster_types)
    {
        if (interpolaters.find(tx) == interpolaters.end())
        {
            ROS_ERROR("Unable to find thrust map for thruster type %s.", tx.c_str());
            exit(1);
        }

        _1D::MonotonicInterpolator<float> &interp = interpolaters[tx];
        float min = *min_element(std::begin(config.thrust_maps[tx].thrust), std::end(config.thrust_maps[tx].thrust));
        float max = *max_element(std::begin(config.thrust_maps[tx].thrust), std::end(config.thrust_maps[tx].thrust));

        thrusters.push_back(Thruster(interp, min, max));
    }

    ROS_INFO("Loaded Thruster configuration.");

    // initialize ros node stuff
    ros::init(argc, argv, "pecka_pvmc");
    ros::NodeHandle nh("pecka_tvmc");

    msg = new std_msgs::Int32MultiArray;
    msg->data.resize(config.spec.number_of_thrusters);

    // ensure thrust_vector, pwm_vector is non empty
    thrust_vector = (float *)malloc(sizeof(float) * config.spec.number_of_thrusters);

    for (int i = 0; i < config.spec.number_of_thrusters; i++)
        thrust_vector[i] = 0;

    // create publisher and subscriber
    pub = new ros::Publisher;
    *pub = nh.advertise<std_msgs::Int32MultiArray>("/control/pwm", 50);
    ros::Subscriber sub = nh.subscribe<std_msgs::Float32MultiArray>("thrust", 1, thrustCallback);

    // register sigint handler
    signal(SIGINT, handle_signint);

    ros::spin();
    return 0;
}
