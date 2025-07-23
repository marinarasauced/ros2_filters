#ifndef _FILTERS_UKF_CORE_IMPL_HPP
#define _FILTERS_UKF_CORE_IMPL_HPP

#include "filters_ukf_core/core.hpp"

namespace filters_ukf_core
{

template<int N_X, int N_U, int N_W>
UKFCore<N_X, N_U, N_W>::UKFCore(
    const std::shared_ptr<ModelProcess<N_X, N_U, N_W>>& model_process,
    const VectorX& x0,
    const MatrixX& P0,
    const MatrixW& Q0,
    double alpha,
    double beta,
    double kappa
) :
    x_(x0),
    P_(P0),
    Q_(Q0),
    alpha_(alpha),
    beta_(beta),
    kappa_(kappa),
    model_process_(model_process)
{
    lambda_ = alpha_ * alpha_ * (N_AUG + kappa_) - N_AUG;
    gamma_ = std::sqrt(N_AUG + lambda_);
    
    computeWeights();
}


template<int N_X, int N_U, int N_W>
typename UKFCore<N_X, N_U, N_W>::VectorX UKFCore<N_X, N_U, N_W>::computeRK4Step(
    const VectorX& x,
    const VectorU& u,
    const VectorW& w,
    double t,
    double dt
) {
    const auto& f = [this](
        const VectorX& x,
        const VectorU& u,
        const VectorW& w,
        double t
    ) {
        return model_process_->f(x, u, w, t);
    };
    VectorX k1 = f(x, u, w, t) * dt;
    VectorX k2 = f(x + 0.5 * k1, u, w, t) * dt;
    VectorX k3 = f(x + 0.5 * k2, u, w, t) * dt;
    VectorX k4 = f(x + k3, u, w, t) * dt;

    VectorX xk = x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
    return xk;
}


template<int N_X, int N_U, int N_W>
void UKFCore<N_X, N_U, N_W>::predictMP(
    const VectorU& u,
    double t,
    double dt
) {
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Matrix<double, N_AUG, N_SIGMA> x_sigma;
    computeAugmentedSigmaPoints(x_sigma);

    Eigen::Matrix<double, N_X, N_SIGMA> x_pred;
    for (int i = 0; i < N_SIGMA; ++i) {
        VectorX x_i = x_sigma.col(i).template head<N_X>();
        VectorW w_i = x_sigma.col(i).template tail<N_W>();
        x_pred.col(i) = computeRK4Step(x_i, u, w_i, t, dt);
    }

    x_.setZero();
    for (int i = 0; i < N_SIGMA; ++i) {
        x_ += weights_mean_(i) * x_pred.col(i);
    }

    P_.setZero();
    for (int i = 0; i < N_SIGMA; ++i) {
        auto dx = x_pred.col(i) - x_;
        P_ += weights_cov_(i) * dx * dx.transpose();
    }
}


template<int N_X, int N_U, int N_W>
template<int N_Z>
void UKFCore<N_X, N_U, N_W>::addMeasurementModel(
    std::shared_ptr<ModelMeasurement<N_X, N_Z>> model_measurement
) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto wrapper = std::make_shared<Wrapper<N_Z>>(model_measurement);
    models_measurement_.push_back(wrapper);
}


template<int N_X, int N_U, int N_W>
template<int N_Z>
void UKFCore<N_X, N_U, N_W>::updateMM(
    std::shared_ptr<ModelMeasurement<N_X, N_Z>> model_measurement,
    Eigen::Matrix<double, N_Z, 1>& z
) {
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Matrix<double, N_AUG, N_SIGMA> x_sigma;
    computeAugmentedSigmaPoints(x_sigma);

    Eigen::Matrix<double, N_Z, N_SIGMA> z_sigma;
    for (int i = 0; i < N_SIGMA; ++i) {
        VectorX x_i = x_sigma.col(i).template head<N_X>();
        z_sigma.col(i) = model_measurement->h(x_i);
    }

    Eigen::Matrix<double, N_Z, 1> z_pred = Eigen::Matrix<double, N_Z, 1>::Zero();
    for (int i = 0; i < N_SIGMA; ++i) {
        z_pred += weights_mean_(i) * z_sigma.col(i);
    }

    Eigen::Matrix<double, N_Z, N_Z> S = Eigen::Matrix<double, N_Z, N_Z>::Zero();
    Eigen::Matrix<double, N_X, N_Z> T = Eigen::Matrix<double, N_X, N_Z>::Zero();
    for (int i = 0; i < N_SIGMA; ++i) {
        Eigen::Matrix<double, N_Z, 1> dz = z_sigma.col(i) - z_pred;
        VectorX dx = x_sigma.col(i).template head<N_X>() - x_;
        S += weights_cov_(i) * dz * dz.transpose();
        T += weights_cov_(i) * dx * dz.transpose();
    }
    S += model_measurement->R();

    Eigen::Matrix<double, N_X, N_Z> K = T * S.inverse();
    Eigen::Matrix<double, N_Z, 1> y = z - z_pred;
    x_ += K * y;
    P_ -= K * S * K.transpose();

    model_measurement->setInnovation(y);
}


template<int N_X, int N_U, int N_W>
typename UKFCore<N_X, N_U, N_W>::VectorX UKFCore<N_X, N_U, N_W>::getX() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return x_;
}


template<int N_X, int N_U, int N_W>
typename UKFCore<N_X, N_U, N_W>::MatrixX UKFCore<N_X, N_U, N_W>::getP() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return P_;
}


template<int N_X, int N_U, int N_W>
typename UKFCore<N_X, N_U, N_W>::MatrixW UKFCore<N_X, N_U, N_W>::getQ() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return Q_;
}


template<int N_X, int N_U, int N_W>
void UKFCore<N_X, N_U, N_W>::computeWeights(
) {
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
void UKFCore<N_X, N_U, N_W>::computeAugmentedSigmaPoints(
    Eigen::Matrix<double, N_AUG, N_SIGMA>& x_sigma
) {
    using VectorA = Eigen::Matrix<double, N_AUG, 1>;
    using MatrixA = Eigen::Matrix<double, N_AUG, N_AUG>;

    VectorA x_aug = VectorA::Zero();
    MatrixA P_aug = MatrixA::Zero();

    x_aug.template head<N_X>() = x_;
    P_aug.template topLeftCorner<N_X, N_X>() = P_;
    P_aug.template bottomRightCorner<N_W, N_W>() = Q_;

    Eigen::LLT<MatrixA> llt(P_aug);
    MatrixA L = llt.matrixL();

    x_sigma.col(0) = x_aug;
    for (int i = 0; i < N_AUG; ++i) {
        x_sigma.col(i + 1) = x_aug + gamma_ * L.col(i);
        x_sigma.col(i + 1 + N_AUG) = x_aug - gamma_ * L.col(i);
    }
}

} // namespace filters_ukf_core

#endif // _FILTERS_UKF_CORE_IMPL_HPP
