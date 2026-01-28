#include "thruster_config.h"

#include <fstream>
#include <cstdlib>

#include "json.hpp"
#include "csv.h"

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using json = nlohmann::json;

ThrusterConfig loadThrusterConfig()
{
    // ROS 2 package path
    std::string path =
        ament_index_cpp::get_package_share_directory("pecka_tvmc") +
        "/config/config.json";

    std::ifstream f(path);

    if (!f.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("thruster_config"),
                     "Unable to open config file: %s",
                     path.c_str());
        std::exit(1);
    }

    ThrusterConfig config;

    json file = json::parse(f);

    if (!file.contains("thrusterSpec"))
    {
        RCLCPP_ERROR(rclcpp::get_logger("thruster_config"),
                     "Unable to find thruster spec.");
        std::exit(1);
    }

    if (!file.contains("thrustVectors"))
    {
        RCLCPP_ERROR(rclcpp::get_logger("thruster_config"),
                     "Unable to find thrust vectors.");
        std::exit(1);
    }

    if (!file.contains("pwmThrustMaps"))
    {
        RCLCPP_ERROR(rclcpp::get_logger("thruster_config"),
                     "Unable to find thrust maps.");
        std::exit(1);
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
    for (auto &type : spec.at("thrustMaps").items())
        config.spec.thruster_types.push_back(type.value().get<std::string>());

    // read thruster vectors
    config.vectors.surge = vectors.at("surge").get<std::vector<float>>();
    config.vectors.pitch = vectors.at("pitch").get<std::vector<float>>();
    config.vectors.roll  = vectors.at("roll").get<std::vector<float>>();
    config.vectors.yaw   = vectors.at("yaw").get<std::vector<float>>();
    config.vectors.heave = vectors.at("heave").get<std::vector<float>>();
    config.vectors.sway  = vectors.at("sway").get<std::vector<float>>();

    // read thrust maps
    for (auto &map : thrust_maps.items())
    {
        PWMThrustMap m;

        std::string tmpath =
            ament_index_cpp::get_package_share_directory("pecka_tvmc") +
            "/config/" + map.value().get<std::string>();

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
