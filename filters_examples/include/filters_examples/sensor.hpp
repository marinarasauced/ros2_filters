#ifndef _FILTERS_EXAMPLES_SENSOR_NODE_HPP
#define _FILTERS_EXAMPLES_SENSOR_NODE_HPP

#include <cmath>
#include <random>

#include <rclcpp/rclcpp.hpp>

#include "filters_core_kf/core.hpp"
#include "filters_examples/msg/measurement.hpp"
#include "filters_examples/msg/state.hpp"

namespace filters_examples
{

class NodeSensor : public rclcpp::Node
{
public:
    NodeSensor();

private:
    rclcpp::Publisher<filters_examples::msg::State>::SharedPtr publisher_x_;
    rclcpp::Publisher<filters_examples::msg::Measurement>::SharedPtr publisher_z_;
    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::VectorXd x_;
    
    void handle_timer_();

    std::default_random_engine rng_;
    std::normal_distribution<double> noise_{0.0, 0.1};
};

} // end namespace filters_examples

#endif // _FILTERS_EXAMPLES_SENSOR_NODE_HPP