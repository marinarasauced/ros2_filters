#ifndef _FILTERS_KF_CORE_IMPL_HPP
#define _FILTERS_KF_CORE_IMPL_HPP

#include "filters_kf_core/core.hpp"

namespace filters_kf_core
{

template<typename ProcessT, typename MeasurementT>
FilterKF<ProcessT, MeasurementT>::FilterKF(
    const typename FilterKF<ProcessT, MeasurementT>::VectorX& x0,
    const typename FilterKF<ProcessT, MeasurementT>::MatrixXX& P0,
    const rclcpp::Time& tic
) :
    filters_base::Filter<ProcessT, MeasurementT>(x0, P0, tic)
{
}


template<typename ProcessT, typename MeasurementT>
void FilterKF<ProcessT, MeasurementT>::predict(
    const std::shared_ptr<filters_base::ModelProcess<ProcessT>>& mp,
    const typename FilterKF<ProcessT, MeasurementT>::VectorU& u,
    const double t,
    const double dt
) {   
    MatrixXX A = mp->A(this->x_, t, dt);
    MatrixXX G = mp->G(VectorW::Zero(), t, dt);
    MatrixWW Q = mp->Q(t, dt);

    VectorX x_minus = this->rk4step(mp, this->x_, u, VectorW::Zero(), t, dt);
    MatrixXX P_minus = A * this->P_ * A.transpose() + G * Q * G.transpose();

    this->x_ = x_minus;
    this->P_ = P_minus;
}


template<typename ProcessT, typename MeasurementT>
void FilterKF<ProcessT, MeasurementT>::update(
    const std::shared_ptr<filters_base::ModelMeasurement<MeasurementT>>& mm,
    const typename MeasurementT::VectorZ& z,
    double t
) {
    typename MeasurementT::MatrixZX H = mm->H(this->x_);
    typename MeasurementT::MatrixZZ R = mm->R();

    typename MeasurementT::MatrixZZ S = H * this->P_ * H.transpose() + R;
    typename MeasurementT::MatrixZX::TransposeReturnType K = this->P_ * H.transpose() * S.inverse();

    VectorX x_minus = this->x_;
    typename MeasurementT::VectorZ z_minus = H * x_minus;
    typename MeasurementT::VectorZ y = z - z_minus

    this->x_ = x_minus + K * y;
    this->P_ = (MatrixXX::Identity() - K * H) * this->P_;
}

} // end namespace filters_kf_core

#endif // _FILTERS_KF_CORE_IMPL_HPP
