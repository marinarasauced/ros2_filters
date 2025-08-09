#ifndef _FILTERS_EXAMPLES_KF_NODE_HPP
#define _FILTERS_EXAMPLES_KF_NODE_HPP

#include <random>

#include <rclcpp/rclcpp.hpp>

#include "filters_core_kf/core.hpp"
#include "filters_examples/model_measurement.hpp"
#include "filters_examples/model_process.hpp"
#include "filters_examples/msg/state.hpp"
#include "filters_examples/msg/measurement.hpp"

namespace filters_examples
{

class NodeFilterKF : public rclcpp::Node
{
public:
    using FilterT = filters_core_kf::FilterCore<ProcessT>;

    NodeFilterKF();

private:
    rclcpp::Publisher<filters_examples::msg::State>::SharedPtr publisher_x_;
    rclcpp::Subscription<filters_examples::msg::Measurement>::SharedPtr subscription_z_;
    rclcpp::TimerBase::SharedPtr timer_;

    void handle_publisher_x_();
    void handle_subscription_z_(const filters_examples::msg::Measurement::SharedPtr msg);
    void handle_timer_();

    std::shared_ptr<FilterT> filter_;
    std::shared_ptr<ModelProcess> model_process_;
    std::shared_ptr<ModelMeasurement> model_measurement_;

    std::default_random_engine rng_;
    std::normal_distribution<double> noise_{0.0, 0.1};
};

} // end namespace filters_examples

#endif // _FILTERS_EXAMPLES_KF_NODE_HPP