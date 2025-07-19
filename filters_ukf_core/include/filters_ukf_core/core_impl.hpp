#ifndef _FILTERS_UKF_CORE_IMPL_HPP
#define _FILTERS_UKF_CORE_IMPL_HPP

#include "filters_ukf_core/core.hpp"

namespace filters_ukf_core
{
template<int N_X, int N_U, int N_W>
UKFCore<N_X, N_U, N_W>::UKFCore(
    std::shared_ptr<model_process> model_process,
    const MatrixX& P,
    const VectorW& w_mean,
    const MatrixW& Q
) : 
    x_(VectorX::Zero()),
    P_(P),
    w_mean_(w_mean),
    Q_(Q),
    u_(VectorU::Zero()),
    model_process_(std::move(model_process))
{
    lambda_ = alpha_ * alpha_ * (N_AUG - N_X) - N_AUG;
    gamma_ = std::sqrt(N_AUG + lambda_);

    (void) _compute_weights();
}


template<int N_X, int N_U, int N_W>
typename UKFCore<N_X, N_U, N_W>::VectorX UKFCore<N_X, N_U, N_W>::rk4_step(
    const VectorX& x,
    const VectorU& u,
    const VectorW& w,
    double dt
) {
    const auto& f = [this](
        const VectorX& x,
        const VectorU& u,
        const VectorW& w
    ) {
        return this->model_process_->f(x, u, w);
    };
    VectorX k1 = f(x, u, w) * dt;
    VectorX k2 = f(x + 0.5 * k1, u, w) * dt;
    VectorX k3 = f(x + 0.5 * k2, u, w) * dt;
    VectorX k4 = f(x + k3, u, w) * dt;

    VectorX xk = x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
    return xk;
}


template<int N_X, int N_U, int N_W>
void UKFCore<N_X, N_U, N_W>::predict(
    double dt
) {
    std::lock_guard<std::mutex> lock(mutex_);

    using MatrixPredX = Eigen::Matrix<double, N_X, N_SIGMA>;

    MatrixSigma sigma_points = _generate_sigma_points(x_, P_);
    MatrixPredX x_pred;

    for (int i = 0; i < N_SIGMA; ++i) {
        VectorX x_i = sigma_points.template block<N_X, 1>(0, i);
        VectorW w_i = sigma_points.template block<N_W, 1>(N_X, i);
        x_pred.col(i) = rk4_step(x_i, u_, w_i, dt);
    }

    x_.setZero();
    for (int i = 0; i < N_SIGMA; ++i) {
        x_ += weights_mean_[i] * x_pred.col(i);
    }

    P_.setZero();
    for (int i = 0; i < N_SIGMA; ++i) {
        VectorX dx = x_pred.col(i) - x_;
        P_ += weights_cov_[i] * dx * dx.transpose();
    }
}


template<int N_X, int N_U, int N_W>
template<int N_Z>
std::shared_ptr<ModelMeasurement<N_X, N_Z>> UKFCore<N_X, N_U, N_W>::add_model_measurement(
    std::shared_ptr<ModelMeasurement<N_X, N_Z>> model_measurement)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto wrapper = std::make_shared<WrapperMM<N_Z>>(model_measurement);
    models_measurement_.push_back(wrapper);
    return model_measurement;
}


template<int N_X, int N_U, int N_W>
template<int N_Z>
void UKFCore<N_X, N_U, N_W>::update(
    const std::shared_ptr<ModelMeasurement<N_X, N_Z>>& model_measurement,
    const Eigen::Matrix<double, N_Z, 1>& z
) {
    std::lock_guard<std::mutex> lock(mutex_);

    using MatrixPredX = Eigen::Matrix<double, N_X, N_SIGMA>;
    using MatrixPredZ = Eigen::Matrix<double, N_Z, N_SIGMA>;

    MatrixSigma sigma_points = _generate_sigma_points(x_, P_);
    MatrixPredX x_sigma = sigma_points.template topRows<N_X>();
    MatrixPredZ z_sigma;

    for (int i = 0; i < N_SIGMA; ++i) {
        z_sigma.col(i) = model_measurement->h(x_sigma.col(i));
    }

    Eigen::Matrix<double, N_Z, 1> z_pred = Eigen::Matrix<double, N_Z, 1>::Zero();

    for (int i = 0; i < N_SIGMA; ++i) {
        z_pred += weights_mean_[i] * z_sigma.col(i);
    }

    Eigen::Matrix<double, N_Z, N_Z> S = Eigen::Matrix<double, N_Z, N_Z>::Zero();
    Eigen::Matrix<double, N_X, N_Z> T = Eigen::Matrix<double, N_X, N_Z>::Zero();

    for (int i = 0; i < N_SIGMA; ++i) {
        auto dz = z_sigma.col(i) - z_pred;
        auto dx = x_sigma.col(i) - x_;

        S += weights_cov_[i] * dz * dz.transpose();
        T += weights_cov_[i] * dx * dz.transpose();
    }

    S += model_measurement->R();
    S_ = S;

    Eigen::Matrix<double, N_X, N_Z> K = T * S.inverse();
    Eigen::Matrix<double, N_Z, 1> y = z - z_pred;
    y_ = y;

    x_ += K * y;
    P_ -= K * S * K.transpose();
}


template<int N_X, int N_U, int N_W>
typename UKFCore<N_X, N_U, N_W>::VectorX UKFCore<N_X, N_U, N_W>::get_x() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return x_;
}


template<int N_X, int N_U, int N_W>
typename UKFCore<N_X, N_U, N_W>::MatrixX UKFCore<N_X, N_U, N_W>::get_P() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return P_;
}


template<int N_X, int N_U, int N_W>
Eigen::VectorXd UKFCore<N_X, N_U, N_W>::get_y() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return y_;
}


template<int N_X, int N_U, int N_W>
Eigen::MatrixXd UKFCore<N_X, N_U, N_W>::get_S() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return S_;
}


template<int N_X, int N_U, int N_W>
void UKFCore<N_X, N_U, N_W>::set_u(const VectorU& u) {
    std::lock_guard<std::mutex> lock(mutex_);
    u_ = u;
}


template<int N_X, int N_U, int N_W>
void UKFCore<N_X, N_U, N_W>::_compute_weights() {
    weights_mean_.resize(N_SIGMA);
    weights_cov_.resize(N_SIGMA);

    weights_mean_[0] = lambda_ / (N_AUG + lambda_);
    weights_cov_[0] = weights_mean_[0] + (1 - alpha_ * alpha_ + beta_);

    for (int i = 1; i < N_SIGMA; ++i) {
        weights_mean_[i] = 1.0 / (2.0 * (N_AUG + lambda_));
        weights_cov_[i] = weights_mean_[i];
    }
}


template<int N_X, int N_U, int N_W>
Eigen::Matrix<double, N_X + N_W, 2 * (N_X + N_W) + 1> UKFCore<N_X, N_U, N_W>::_generate_sigma_points(
    const VectorX& x,
    const MatrixX& P
) {
    Eigen::Matrix<double, N_AUG, N_SIGMA> sigma_points;

    Eigen::Matrix<double, N_AUG, 1> x_aug;
    x_aug.head(N_X) = x;
    x_aug.tail(N_W) = w_mean_;

    Eigen::Matrix<double, N_AUG, N_AUG> P_aug = Eigen::Matrix<double, N_AUG, N_AUG>::Zero();
    P_aug.topLeftCorner(N_X, N_X) = P;
    P_aug.bottomRightCorner(N_W, N_W) = Q_;

    Eigen::Matrix<double, N_AUG, N_AUG> A = P_aug.llt().matrixL();

    sigma_points.col(0) = x_aug;
    for (int i = 0; i < N_AUG; ++i) {
        sigma_points.col(i + 1) = x_aug + gamma_ * A.col(i);
        sigma_points.col(i + 1 + N_AUG) = x_aug - gamma_ * A.col(i);
    }

    return sigma_points;
}

} // namespace filters_ukf_core

#endif // _FILTERS_UKF_CORE_IMPL_HPP
