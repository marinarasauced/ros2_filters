#ifndef _FILTERS_CORE_KF_IMPL_HPP
#define _FILTERS_CORE_KF_IMPL_HPP

#include "filters_core_kf/core.hpp"

namespace filters_core_kf
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
    MatrixXX A = mp->A(this->x_, t, dt);
    MatrixXX G = mp->G(VectorW::Zero(), t, dt);
    MatrixWW Q = mp->Q(t, dt);

    VectorX x_minus = this->rk4Step(mp, this->x_, u, VectorW::Zero(), t, dt);
    MatrixXX P_minus = A * this->P_ * A.transpose() + G * Q * G.transpose();

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
    typename MeasurementT::MatrixZX H = mm->H(this->x_);
    typename MeasurementT::MatrixZZ R = mm->R();

    typename MeasurementT::MatrixZZ S = H * this->P_ * H.transpose() + R;
    auto K = this->P_ * H.transpose() * S.inverse();

    VectorX x_minus = this->x_;
    typename MeasurementT::VectorZ z_minus = H * x_minus;
    typename MeasurementT::VectorZ y = z - z_minus;

    this->x_ = x_minus + K * y;
    this->P_ = (MatrixXX::Identity() - K * H) * this->P_;
}

} // end namespace filters_core_kf

#endif // _FILTERS_CORE_KF_IMPL_HPP
