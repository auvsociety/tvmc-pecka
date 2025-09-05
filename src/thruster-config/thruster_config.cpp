#include "thruster_config.h"
#include <fstream>
#include "json.hpp"
#include "csv.h"
#include <ros/package.h>
#include <ros/console.h>

using json = nlohmann::json;

ThrusterConfig loadThrusterConfig()
{
    std::string path = ros::package::getPath("pecka_tvmc") + "/config/config.json";
    std::ifstream f(path);

    ThrusterConfig config;

    json file = json::parse(f);

    if (!file.contains("thrusterSpec"))
    {
        ROS_ERROR("Unable to find thruster spec.");
        exit(1);
    }

    if (!file.contains("thrustVectors"))
    {
        ROS_ERROR("Unable to find thrust vectors.");
        exit(1);
    }

    if (!file.contains("pwmThrustMaps"))
    {
        ROS_ERROR("Unable to find thrust maps.");
        exit(1);
    }

    auto spec = file.at("thrusterSpec");
    auto vectors = file.at("thrustVectors");
    auto thrust_maps = file.at("pwmThrustMaps");

    // read params
    config.spec.number_of_thrusters = spec.at("noOfThrusters");
    config.spec.min_thrust = spec.at("minThrust");
    config.spec.max_thrust = spec.at("maxThrust");
    config.spec.full_thrust = spec.at("fullThrust");
    config.pwm_offset = file.at("pwmOffset");

    // read thruster types
    for (auto &type: spec.at("thrustMaps").items())
    config.spec.thruster_types.push_back(type.value().get<std::string>());

    // read thrustered vectors
    config.vectors.surge = vectors.at("surge").get<std::vector<float>>();
    config.vectors.pitch = vectors.at("pitch").get<std::vector<float>>();
    config.vectors.roll = vectors.at("roll").get<std::vector<float>>();
    config.vectors.yaw = vectors.at("yaw").get<std::vector<float>>();
    config.vectors.heave = vectors.at("heave").get<std::vector<float>>();
    config.vectors.sway = vectors.at("sway").get<std::vector<float>>();

    // read thrust maps
    for (auto &map : thrust_maps.items())
    {
        PWMThrustMap m;

        std::string tmpath = ros::package::getPath("pecka_tvmc") + "/config/" + map.value().get<std::string>();
        io::CSVReader<2> csv(tmpath);
        int pwm;
        float thrust;

        while (csv.read_row(pwm, thrust))
        {
            m.thrust.push_back(thrust);
            m.pwm.push_back(pwm);
        }

        config.thrust_maps[map.key()] = m;
    }

    return config;
}
