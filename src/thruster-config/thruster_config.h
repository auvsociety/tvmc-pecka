#ifndef THRUSTER_CONFIG_H
#define THRUSTER_CONFIG_H

#include <map>
#include <vector>
#include <chrono>
#include <ros/console.h>

typedef struct PWMThrustMap {
    std::vector<float> thrust;
    std::vector<int> pwm;
} PWMThrustMap;

typedef struct ThrusterSpec
{
    int number_of_thrusters;
    int min_thrust;
    int max_thrust;
    int full_thrust;

    std::vector<std::string> thruster_types;

    // int zero_thrust_pwm;
    // int min_pwm;
    // int max_pwm;
    // std::vector<float> thrust_map_thrust;
    // std::vector<int> thrust_map_pwm;
} ThrusterSpec;

typedef struct ThrustVectors
{
    std::vector<float> surge;
    std::vector<float> heave;
    std::vector<float> sway;
    std::vector<float> roll;
    std::vector<float> pitch;
    std::vector<float> yaw;

} ThrustVectors;

typedef struct ThrusterConfig
{
    ThrusterSpec spec;
    ThrustVectors vectors;
    std::map<std::string, PWMThrustMap> thrust_maps;
    int pwm_offset;

} ThrusterConfig;

ThrusterConfig loadThrusterConfig();

#endif