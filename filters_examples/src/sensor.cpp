
#include "filters_examples/sensor.hpp"

namespace filters_examples
{
    
NodeSensor::NodeSensor(
) :
    Node("kf_example_fake"),
    x_(Eigen::VectorXd::Zero(2))
{
    publisher_x_ = this->create_publisher<filters_examples::msg::State>(
        "kf/example/fake/x",
        10
    );

    publisher_z_ = this->create_publisher<filters_examples::msg::Measurement>(
        "kf/example/fake/z",
        10
    );

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(
            &NodeSensor::handle_timer_,
            this
        )
    );
}


void NodeSensor::handle_timer_(
) {
    rclcpp::Time now = this->now();
    double t = now.seconds();

    x_(0) = std::sin(t);
    x_(1) = std::cos(t);

    filters_examples::msg::State msg_x;
    msg_x.header.stamp = now;
    msg_x.x1 = x_(0);
    msg_x.x2 = x_(1);
    publisher_x_->publish(msg_x);

    filters_examples::msg::Measurement msg_z;
    msg_z.header.stamp = now;
    msg_z.z1 = x_(1) + noise_(rng_);
    publisher_z_->publish(msg_z);
}

} // end namespace filters_examples


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<filters_examples::NodeSensor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}