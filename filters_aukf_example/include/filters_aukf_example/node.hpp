#ifndef _FILTERS_AUKF_EXAMPLE_WRAP_MODEL_HPP
#define _FILTERS_AUKF_EXAMPLE_WRAP_MODEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

#include "filters_msgs/msg/filters_state.hpp"
#include "filters_msgs/msg/filters_innovation.hpp"
#include "filters_aukf_core/core.hpp"
#include "filters_aukf_example/parameters.hpp"
#include "filters_aukf_example/wrap_measurement.hpp"
#include "filters_aukf_example/wrap_process.hpp"


namespace filters_aukf_example
{

class UKFNode : public rclcpp::Node
{
public:
    static constexpr int N_X = 2;
    static constexpr int N_U = 0;
    static constexpr int N_W = 1;

    using UKF = filters_aukf_core::AUKFCore<N_X, N_U, N_W>;
    using VectorX = typename UKF::VectorX;
    using MatrixX = typename UKF::MatrixX;
    using VectorW = typename UKF::VectorW;
    using MatrixW = typename UKF::MatrixW;

    UKFNode();

private:
    BlueROV2Parameters params_;

    std::shared_ptr<F<N_X, N_U, N_W>> f_;
    std::shared_ptr<H2<N_X, 1>> h2_;
    std::shared_ptr<UKF> ukf_;

    rclcpp::Publisher<filters_msgs::msg::FiltersState>::SharedPtr _publisher_x;
    rclcpp::Publisher<filters_msgs::msg::FiltersInnovation>::SharedPtr _publisher_y;
    rclcpp::Publisher<filters_msgs::msg::FiltersState>::SharedPtr _publisher_z;

    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr _subscription_z2;
    void _handle_subscription_z2(const sensor_msgs::msg::FluidPressure::SharedPtr msg);

    rclcpp::Time tic;
};

} // namespace filters_aukf_example

#endif // _FILTERS_AUKF_EXAMPLE_WRAP_MODEL_HPP
