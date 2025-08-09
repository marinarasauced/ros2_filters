#ifndef _FILTERS_CORE_EKF_IMPL_HPP
#define _FILTERS_CORE_EKF_IMPL_HPP

#include "filters_core_ekf/core.hpp"

namespace filters_core_ekf
{

template<typename ProcessT>
FilterCore<ProcessT>::FilterCore(
    const typename FilterCore<ProcessT>::VectorX& x0,
    const typename FilterCore<ProcessT>::MatrixXX& P0,
    const rclcpp::Time& tic
) :
    filters_base::FilterBase<ProcessT>(x0, P0, tic)
{
}


template<typename ProcessT>
void FilterCore<ProcessT>::predict(
    const std::shared_ptr<filters_base::ModelProcess<ProcessT>>& mp,
    const typename FilterCore<ProcessT>::VectorU& u,
    const double t,
    const double dt
) {
    MatrixXX F = mp->F(this->x_, t, dt);
    MatrixXX W = mp->W(VectorW::Zero(), t, dt);
    MatrixWW Q = mp->Q(t, dt);

    VectorX x_minus = this->rk4Step(mp, this->x_, u, VectorW::Zero(), t, dt);
    MatrixXX P_minus = F * this->P_ * F.transpose() + W * Q * W.transpose();

    this->x_ = x_minus;
    this->P_ = P_minus;
}


template<typename ProcessT>
template<typename MeasurementT>
void FilterCore<ProcessT>::update(
    const std::shared_ptr<filters_base::ModelMeasurement<MeasurementT>>& mm,
    const typename MeasurementT::VectorZ& z,
    double t
) {
    VectorX x_minus = this->x_;

    typename MeasurementT::MatrixZX H = mm->H(x_minus);
    typename MeasurementT::MatrixZZ R = mm->R();

    typename MeasurementT::MatrixZZ S = H * this->P_ * H.transpose() + R;
    auto K = this->P_ * H.transpose() * S.inverse();

    typename MeasurementT::VectorZ z_minus = mm->h(x_minus);
    typename MeasurementT::VectorZ y = z - z_minus;

    this->x_ = x_minus + K * y;
    this->P_ = (MatrixXX::Identity() - K * H) * this->P_;
}

} // end namespace filters_core_ekf

#endif // _FILTERS_CORE_EKF_IMPL_HPP
