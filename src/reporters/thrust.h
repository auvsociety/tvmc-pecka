#ifndef THRUST_REPORTER_H
#define THRUST_REPORTER_H

#include <rclcpp/rclcpp.hpp>

#define THRUST_REPORT_RATE_US 100000

namespace ThrustReporter
{
    /**
     * Initializes the Thruster Control Node
     */
    void init(rclcpp::Node::SharedPtr node);

    /**
     * Reports the corresponding thrust values for the provided thrust vector.
     *
     * @param thrust_vector List of float values representing the thrust of each thruster.
     */
    void report(float *thrust_vector);

    /**
     * Kills the Thruster Control Node
     */
    void kill();

    // for backwards-compatibility

    /**
     * Force re-publishes the thrust values.
     * You shouldn't have to do this normally, it should be done on its own.
     */
    void refresh();

    /**
     * Alias for kill().
     * Provided for backwards-compatibility with ThrusterController.
     */
    void shutdown();

    /**
     * Alias for report().
     * Provided for backwards-compatibility with ThrusterController.
     *
     * Reports the corresponding thrust values for the provided thrust vector.
     *
     * @param thrust_vector List of float values representing the thrust of each thruster.
     */
    void writeThrusterValues(float *thrust_vector);
}

#endif // THRUST_REPORTER_H
