#ifndef _FILTERS_BASE_IMPL_HPP
#define _FILTERS_BASE_IMPL_HPP

#include "filters_base/base.hpp"

namespace filters_base
{

template<typename ProcessT>
Filter<ProcessT>::Filter(
    const typename Filter<ProcessT>::VectorX& x0,
    const typename Filter<ProcessT>::MatrixXX& P0,
    const rclcpp::Time& tic
) :
    x_(x0),
    P_(P0),
    tic(tic)
{
}


template<typename ProcessT>
typename Filter<ProcessT>::VectorX Filter<ProcessT>::rk4Step(
    const std::shared_ptr<ModelProcess<ProcessT>> model_process,
    const typename Filter<ProcessT>::VectorX& x,
    const typename Filter<ProcessT>::VectorU& u,
    const typename Filter<ProcessT>::VectorW& w,
    double t,
    double dt
) const {
    auto sysdyn = [&](const VectorX xk, double tk) -> VectorX {
        return model_process->f(xk, u, w, tk, dt);
    };

    VectorX k1 = sysdyn(x, t);
    VectorX k2 = sysdyn(x + 0.5 * dt * k1, t + 0.5 * dt);
    VectorX k3 = sysdyn(x + 0.5 * dt * k2, t + 0.5 * dt);
    VectorX k4 = sysdyn(x + dt * k3, t + dt);

    VectorX xk = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    return xk;
}


template<typename ProcessT>
void Filter<ProcessT>::enQueue(
    const std::shared_ptr<MeasurementInterface>& measurement
) {
    queue_.push(measurement);
}


template<typename ProcessT>
void Filter<ProcessT>::deQueue(
    const std::shared_ptr<ModelProcess<ProcessT>>& model_process,
    const rclcpp::Time toc
) {
    while (!queue_.empty() && rclcpp::ok()) {
        auto& msg = queue_.top();
        const rclcpp::Time stamp = msg->stamp();
        if (stamp > toc) { break; }

        const double dt = (stamp - tic).seconds();
        if (dt > 0.0) {
            this->predict(model_process, ProcessT::VectorU::Zero(), tic.seconds(), dt);
            tic = stamp;
        }

        msg->dispatch();
        queue_.pop();
    }

    const double dt = (toc - tic).seconds();
    if (dt > 0.0) {
        this->predict(model_process, ProcessT::VectorU::Zero(), toc.seconds(), dt);
        tic = toc;
    }
}


template<typename ProcessT>
void Filter<ProcessT>::addModelProcess(
    const std::shared_ptr<ModelProcess<ProcessT>>& model_process
) {
    if (!model_process) {
        throw std::runtime_error("Model process cannot be null.");
    }

    models_process_.push_back(model_process);
}


template<typename ProcessT>
template<typename MeasurementT>
void Filter<ProcessT>::addModelMeasurement(
    const std::shared_ptr<ModelMeasurement<MeasurementT>>& model_measurement
) {
    if (!model_measurement) {
        throw std::runtime_error("Model measurement cannot be null.");
    }

    models_measurement_.push_back(model_measurement);
}


template<typename ProcessT>
void Filter<ProcessT>::removeModelProcess(
    const std::shared_ptr<ModelProcess<ProcessT>>& model_process
) {
    auto it = std::remove(models_process_.begin(), models_process_.end(), model_process);
    if (it != models_process_.end()) {
        models_process_.erase(it, models_process_.end());
    } else {
        throw std::runtime_error("Model process not found.");
    }
}


template<typename ProcessT>
template<typename MeasurementT>
void Filter<ProcessT>::removeModelMeasurement(
    const std::shared_ptr<ModelMeasurement<MeasurementT>>& model_measurement
) {
    auto it = std::remove(models_measurement_.begin(), models_measurement_.end(), model_measurement);
    if (it != models_measurement_.end()) {
        models_measurement_.erase(it, models_measurement_.end());
    } else {
        throw std::runtime_error("Model measurement not found.");
    }
}

} // end namespace filters_base

#endif // _FILTERS_BASE_IMPL_HPP
