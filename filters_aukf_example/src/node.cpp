
#include "filters_aukf_example/node.hpp"

namespace filters_aukf_example
{

UKFNode::UKFNode() : Node("ukf_node")
{
    auto model_process = std::make_shared<F<N_X, N_U, N_W>>();
    f_ = model_process;

    Eigen::Matrix<double, N_X, 1> x = Eigen::Matrix<double, N_X, 1>::Zero();
    Eigen::Matrix<double, N_X, N_X> P = Eigen::Matrix<double, N_X, N_X>::Identity() * 1.0;
    // Eigen::Matrix<double, N_W, 1> w_mean = Eigen::Matrix<double, N_W, 1>::Zero();
    Eigen::Matrix<double, N_W, N_W> Q = Eigen::Matrix<double, N_W, N_W>::Identity() * 1.0;

    Q(0, 0) = 0.2;

    _publisher_x = this->create_publisher<filters_msgs::msg::FiltersState>(
        "filters/ukf/state",
        10
    );

    _publisher_y = this->create_publisher<filters_msgs::msg::FiltersInnovation>(
        "filters/ukf/innov",
        10
    );

    _publisher_z = this->create_publisher<filters_msgs::msg::FiltersState>(
        "filters/ukf/state_raw",
        10
    );

    h2_ = std::make_shared<H2<N_X, 1>>(params_, 100.0);

    ukf_ = std::make_shared<UKF>(f_, x, P, Q);
    ukf_->addMeasurementModel<1>(h2_);

    _subscription_z2 = this->create_subscription<sensor_msgs::msg::FluidPressure>(
        "pressure",
        10,
        std::bind(
            &UKFNode::_handle_subscription_z2,
            this,
            std::placeholders::_1
        )
    );

    tic = this->now();
}


void UKFNode::_handle_subscription_z2(
    const sensor_msgs::msg::FluidPressure::SharedPtr msg
){
    rclcpp::Time toc = this->now();
    double dt = (toc - tic).seconds();
    if (dt < 0.0) { return; }
    tic = toc;

    Eigen::Matrix<double, N_U, 1> u;
    double t = 0.0;
    ukf_->predictMP(u, t, dt);

    Eigen::Matrix<double, 1, 1> z2;
    z2(0) = msg->fluid_pressure;

    ukf_->updateMM<1>(h2_, z2);

    auto x = ukf_->getX();
    auto P = ukf_->getP();

    filters_msgs::msg::FiltersState msg_state;
    // filters_msgs::msg::FiltersInnovation msg_innov;

    msg_state.header.stamp = this->now();
    // msg_innov.header.stamp = this->now();

    msg_state.state.resize(N_X);
    msg_state.cov.resize(N_X * N_X);
    // msg_innov.innov.resize(y.rows());
    // msg_innov.cov.resize(S.rows() * 2);

    for (int i = 0; i < N_X; ++i) {
        msg_state.state[i] = x(i);
    }

    for (int i = 0; i < N_X; ++i) {
        for (int j = 0; j < N_X; ++j) {
            msg_state.cov[i * N_X + j] = P(i, j);
        }
    }

    filters_msgs::msg::FiltersState msg_state_raw;
    msg_state_raw.header.stamp = this->now();
    msg_state_raw.state.resize(N_X);

    msg_state_raw.state[0] = (-params_.z2.pressure_atm + msg->fluid_pressure) / (params_.z2.rho * params_.gravity);
    _publisher_z->publish(msg_state_raw);
    // for (int i = 0; i < y.rows(); ++i) {
    //     msg_innov.innov[i] = y(i);
    //     msg_innov.cov[2 * i + 0] = 2.0 * std::sqrt(S(i, i));
    //     msg_innov.cov[2 * i + 1] = -2.0 * std::sqrt(S(i, i));
    // }

    _publisher_x->publish(msg_state);
    // _publisher_y->publish(msg_innov);
}

} // namespace filters_aukf_example


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<filters_aukf_example::UKFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
