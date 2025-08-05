#ifndef _FILTERS_KF_EXAMPLE_NODE_HPP
#define _FILTERS_KF_EXAMPLE_NODE_HPP

#include <random>

#include <rclcpp/rclcpp.hpp>

#include "filters_kf_core/core.hpp"
#include "filters_kf_example/model_measurement.hpp"
#include "filters_kf_example/model_process.hpp"
#include "filters_kf_example/msg/fake_x.hpp"
#include "filters_kf_example/msg/fake_z.hpp"

namespace filters_kf_example
{

class KFExampleNode : public rclcpp::Node
{
public:
    using FilterT = filters_kf_core::FilterKF<KFExampleProcessT>;

    KFExampleNode();

private:
    rclcpp::Publisher<filters_kf_example::msg::FakeX>::SharedPtr publisher_x_;
    rclcpp::Subscription<filters_kf_example::msg::FakeZ>::SharedPtr subscription_z_;
    rclcpp::TimerBase::SharedPtr timer_;

    void handle_publisher_x_();
    void handle_subscription_z_(const filters_kf_example::msg::FakeZ::SharedPtr msg);
    void handle_timer_();

    std::shared_ptr<FilterT> filter_;
    std::shared_ptr<KFExampleProcess> model_process_;
    std::shared_ptr<KFExampleMeasurement> model_measurement_;

    std::default_random_engine rng_;
    std::normal_distribution<double> noise_{0.0, 0.1};
};

} // end namespace filters_kf_core

#endif // _FILTERS_KF_EXAMPLE_NODE_HPP
