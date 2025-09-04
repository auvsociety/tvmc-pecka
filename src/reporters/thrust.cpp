#include "ros/ros.h"
#include "thrust.h"
#include "../thruster-config/thruster_config.h"
#include "std_msgs/Float32MultiArray.h"
#include <pthread.h>
#include <unistd.h>
#include <vector>

ros::Publisher *pub;
std_msgs::Float32MultiArray *msg;
ThrusterConfig config;
float *thrust_vector;
pthread_t thread;

void *ThrustReporterThread(void *arg)
{
    while (1)
    {
        // run indefinetly
        ThrustReporter::refresh();
        ros::spinOnce();
        usleep(THRUST_REPORT_RATE_US);
    }
}

void ThrustReporter::init(ros::NodeHandle *nh)
{
    // load thruster config
    config = loadThrusterConfig();

    // ensure thrust_vector is non empty
    thrust_vector = (float *)malloc(sizeof(float) * config.spec.number_of_thrusters);
    for (int i = 0; i < config.spec.number_of_thrusters; i++)
        thrust_vector[i] = 0;

    // create publisher, resize pwm to match no. of thrusters
    pub = new ros::Publisher;
    msg = new std_msgs::Float32MultiArray;

    *pub = nh->advertise<std_msgs::Float32MultiArray>("thrust", 50);
    msg->data.resize(config.spec.number_of_thrusters);

    ROS_INFO("Will start publishing thrust values to %s.", pub->getTopic().c_str());

    // start reporter thread
    pthread_create(&thread, NULL, ThrustReporterThread, NULL);
}

void ThrustReporter::refresh()
{
    for (int i = 0; i < config.spec.number_of_thrusters; i++)
        msg->data[i] = thrust_vector[i];

    pub->publish(*msg);
}

void ThrustReporter::report(float *tvec)
{
    // copy the vector onto local variable
    std::copy(tvec, tvec + config.spec.number_of_thrusters, thrust_vector);
}

void ThrustReporter::kill()
{
    // stop thread
    pthread_cancel(thread);

    // delete node
    delete pub;
    delete msg;

    // free mallocated vector
    free(thrust_vector);
}

// backwards-compatibility

void ThrustReporter::writeThrusterValues(float *thrust_vector)
{
    return ThrustReporter::report(thrust_vector);
}

void ThrustReporter::shutdown()
{
    return ThrustReporter::kill();
}
