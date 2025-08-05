#ifndef _FILTERS_BASE_IMPL_HPP
#define _FILTERS_BASE_IMPL_HPP

#include "filters_base/base.hpp"

namespace filters_base
{

template<typename ProcessT, typename MeasurementT>
Filter<ProcessT, MeasurementT>::Filter(
    const typename Filter<ProcessT, MeasurementT>::VectorX& x0,
    const typename Filter<ProcessT, MeasurementT>::MatrixXX& P0,
    const rclcpp::Time& tic
) :
    x_(x0),
    P_(P0),
    tic(tic)
{
}


template<typename ProcessT, typename MeasurementT>
typename Filter<ProcessT, MeasurementT>::VectorX Filter<ProcessT, MeasurementT>::rk4step(
    const std::shared_ptr<ModelProcess<ProcessT>> model_process,
    const typename Filter<ProcessT, MeasurementT>::VectorX& x,
    const typename Filter<ProcessT, MeasurementT>::VectorU& u,
    const typename Filter<ProcessT, MeasurementT>::VectorW& w,
    double t,
    double dt
) const {
    auto sysdyn = [&](const VecX xk, double tk) -> VecX {
        return model_process->f(xk, u, w, tk);
    };

    VectorX k1 = sysdyn(x, t);
    VectorX k2 = sysdyn(x + 0.5 * dt * k1, t + 0.5 * dt);
    VectorX k3 = sysdyn(x + 0.5 * dt * k2, t + 0.5 * dt);
    VectorX k4 = sysdyn(x + dt * k3, t + dt);

    VectorX xk = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    return xk;
}


template<typename ProcessT, typename MeasurementT>
void Filter<ProcessT, MeasurementT>::enqueue(
    const std::shared_ptr<MeasurementInterface>& measurement
) {
    queue_.push(measurement);
}


template<typename ProcessT, typename MeasurementT>
void Filter<ProcessT, MeasurementT>::dequeue(
    const rclcpp::Time toc
) {
    while (!queue_.empty() && rclcpp::ok()) {
        auto& msg = queue_.top();
        const rclcpp::Time stamp = msg->stamp();
        if (stamp > toc) { break; }

        const double dt = (stamp - tic).seconds();
        if (dt > 0.0) {
            this->predict(model_process_, typename ProcessT::VectorU::Zero(), tic.seconds(), dt);
            tic = stamp;
        }

        msg->dispatch();
        queue_.pop();
    }

    const double dt = (toc - tic).seconds();
    if (dt > 0.0) {
        this->predict(model_process_, typename ProcessT::VectorU::Zero(), toc.seconds(), dt);
        tic = toc;
    }
}

} // end namespace filters_base

#endif // _FILTERS_BASE_IMPL_HPP
