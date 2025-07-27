#ifndef _FILTERS_IEKF_CORE_IMPL_HPP
#define _FILTERS_IEKF_CORE_IMPL_HPP

#include "filters_iekf_core/core.hpp"

namespace filters_iekf_core
{

template<int N_X, int N_U, int N_W>
IEKFCore<N_X, N_U, N_W>::IEKFCore(
    const std::shared_ptr<ModelProcess<N_X, N_U, N_W>>& model_process,
    const VectorX& x0,
    const MatrixX& P0,
    const MatrixW& Q0
) :
    x_(x0),
    P_(P0),
    Q_(Q0),
    model_process_(model_process)
{
}


template<int N_X, int N_U, int N_W>
typename IEKFCore<N_X, N_U, N_W>::VectorX IEKFCore<N_X, N_U, N_W>::computeRK4Step(
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
void IEKFCore<N_X, N_U, N_W>::predictMP(
    const VectorU& u,
    double t,
    double dt
) {
    std::lock_guard<std::mutex> lock(mutex_);

    VectorW w = VectorW::Zero();

    VectorX dx = model_process_->computeRK4Step(x_, u, w, t, dt);

    x_ = LieGroupTraits<N_X>::retract(x_, dx);

    Eigen::Matrix<double, N_X, N_X> jX = model_process_->jacobianX(x_, u, w, t);
    Eigen::Matrix<double, N_X, N_W> fW = model_process_->jacobianW(x_, u, w, t);

    // Propagate covariance
    P_ = jX * P_ * Fx.transpose() + jX * Q_ * jW.transpose();
}


template<int N_X, int N_U, int N_W>
template<int N_Z>
void IEKFCore<N_X, N_U, N_W>::updateMM(
    std::shared_ptr<ModelMeasurement<N_X, N_Z>> model_measurement,
    const Eigen::Matrix<double, N_Z, 1>& z
) {
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Matrix<double, N_Z, 1> z_pred = model_measurement->h(x_);

    Eigen::Matrix<double, N_Z, 1> y = z - z_pred;

    Eigen::Matrix<double, N_Z, N_X> H = model_measurement->jacobianH(x_);
    Eigen::Matrix<double, N_Z, N_Z> R = model_measurement->R();

    Eigen::Matrix<double, N_Z, N_Z> S = H * P_ * H.transpose() + R;

    Eigen::Matrix<double, N_X, N_Z> K = P_ * H.transpose() * S.inverse();

    Eigen::Matrix<double, N_X, 1> dx = K * y;

    x_ = LieGroupTraits<N_X>::retract(x_, dx);

    Eigen::Matrix<double, N_X, N_X> I = Eigen::Matrix<double, N_X, N_X>::Identity();
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();

    model_measurement->setInnovation(y);
}


template<int N_X, int N_U, int N_W>
typename IEKFCore<N_X, N_U, N_W>::VectorX IEKFCore<N_X, N_U, N_W>::getX() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return x_;
}


template<int N_X, int N_U, int N_W>
typename IEKFCore<N_X, N_U, N_W>::MatrixX IEKFCore<N_X, N_U, N_W>::getP() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return P_;
}

} // namespace filters_iekf_core

#endif // _FILTERS_IEKF_CORE_IMPL_HPP
