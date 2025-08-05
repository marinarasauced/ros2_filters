#ifndef _FILTERS_KF_EXAMPLE_FAKE_NODE_HPP
#define _FILTERS_KF_EXAMPLE_FAKE_NODE_HPP

#include <cmath>
#include <random>

#include <rclcpp/rclcpp.hpp>

#include "filters_kf_core/core.hpp"
#include "filters_kf_example/model_measurement.hpp"
#include "filters_kf_example/model_process.hpp"
#include "filters_kf_example/msg/fake_x.hpp"
#include "filters_kf_example/msg/fake_z.hpp"

namespace filters_kf_example
{

class KFExampleFake : public rclcpp::Node
{
public:
    KFExampleFake();

private:
    rclcpp::Publisher<filters_kf_example::msg::FakeX>::SharedPtr publisher_x_;
    rclcpp::Publisher<filters_kf_example::msg::FakeZ>::SharedPtr publisher_z_;
    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::VectorXd x_;
    
    void handle_timer_();

    std::default_random_engine rng_;
    std::normal_distribution<double> noise_{0.0, 0.1};
};

} // end namespace filters_kf_core

#endif // _FILTERS_KF_EXAMPLE_FAKE_NODE_HPP
