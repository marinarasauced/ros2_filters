
#include "filters_kf_example/node.hpp"

namespace filters_kf_example
{

NodeFilter::NodeFilter(
) :
    Node("kf_example_node")
{
    rclcpp::Time now = this->now();

    FilterT::MatrixXX P0;
    P0.setIdentity();

    FilterT::VectorX x0;
    x0(0) = std::sin(now.seconds());
    x0(1) = 0.0;    

    filter_ = std::make_shared<FilterT>(
        x0,
        P0,
        now
    );

    model_process_ = std::make_shared<ModelProcess>();
    model_measurement_ = std::make_shared<ModelMeasurement>();

    filter_->addModelProcess(model_process_);
    filter_->addModelMeasurement<MeasurementT>(model_measurement_);

    publisher_x_ = this->create_publisher<filters_kf_example::msg::FakeX>(
        "kf/example/node/x",
        10
    );


    subscription_z_ = this->create_subscription<filters_kf_example::msg::FakeZ>(
        "kf/example/fake/z",
        10,
        std::bind(
            &NodeFilter::handle_subscription_z_,
            this,
            std::placeholders::_1
        )
    );

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(
            &NodeFilter::handle_timer_,
            this
        )
    );
}


void NodeFilter::handle_publisher_x_(
) {
    auto state = filter_->state();

    filters_kf_example::msg::FakeX msg_x;
    msg_x.header.stamp = this->now();
    msg_x.x1 = state(0);
    msg_x.x2 = state(1);
    publisher_x_->publish(msg_x);
}


void NodeFilter::handle_subscription_z_(
    const filters_kf_example::msg::FakeZ::SharedPtr msg
) {
    auto measurement = std::make_shared<filters_base::Measurement<filters_kf_example::msg::FakeZ>>(
        msg, 
        [this](const filters_kf_example::msg::FakeZ& msg_z) {
            MeasurementT::VectorZ z;
            z(0) = msg_z.z1;
            this->filter_->update<MeasurementT>(model_measurement_, z, this->now().seconds());
        },
        this->now()
    );

    filter_->enQueue(measurement);
}


void NodeFilter::handle_timer_(
) {
    filter_->deQueue(model_process_, this->now());
    handle_publisher_x_();
}


} // end namespace filters_kf_example


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<filters_kf_example::NodeFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}