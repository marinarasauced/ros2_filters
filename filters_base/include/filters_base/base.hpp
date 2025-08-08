#ifndef _FILTERS_BASE_HPP
#define _FILTERS_BASE_HPP

#include <functional>
#include <memory>
#include <queue>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "filters_base/model_measurement.hpp"
#include "filters_base/model_process.hpp"
#include "filters_base/sensors.hpp"
#include "filters_base/types.hpp"

namespace filters_base
{

template<typename ProcessT>
class FilterBaseInterface
{
public:
    using VectorX = typename ProcessT::VectorX;
    using VectorU = typename ProcessT::VectorU;
    using VectorW = typename ProcessT::VectorW;
    using MatrixXX = typename ProcessT::MatrixXX;

    virtual ~FilterBaseInterface() = default;

    virtual void predict(const std::shared_ptr<ModelProcess<ProcessT>>& mp, const typename ProcessT::VectorU& u, const double t, const double dt) = 0;

    template<typename MeasurementT>
    void update(const std::shared_ptr<ModelMeasurement<MeasurementT>>& mm, const typename MeasurementT::VectorZ& z, const double t);
};


template<typename ProcessT>
class FilterBase : public FilterBaseInterface<ProcessT>
{
public:
    using VectorX = typename ProcessT::VectorX;
    using VectorU = typename ProcessT::VectorU;
    using VectorW = typename ProcessT::VectorW;
    using MatrixXX = typename ProcessT::MatrixXX;

    FilterBase(const VectorX& x0, const MatrixXX& P0, const rclcpp::Time& tic);

    VectorX rk4Step(const std::shared_ptr<ModelProcess<ProcessT>> model_process, const VectorX& x, const VectorU& u, const VectorW& w, double t, double dt) const;

    void enQueue(const std::shared_ptr<MeasurementInterface>& measurement);
    void deQueue(const std::shared_ptr<ModelProcess<ProcessT>>& model_process, const rclcpp::Time toc);

    void addModelProcess(const std::shared_ptr<ModelProcess<ProcessT>>& model_process);

    template<typename MeasurementT>
    void addModelMeasurement(const std::shared_ptr<ModelMeasurement<MeasurementT>>& model_measurement);

    void removeModelProcess(const std::shared_ptr<ModelProcess<ProcessT>>& model_process);

    template<typename MeasurementT>
    void removeModelMeasurement(const std::shared_ptr<ModelMeasurement<MeasurementT>>& model_measurement);

    const VectorX& state() const { return x_; }
    const MatrixXX& covariance() const { return P_; }

protected:
    VectorX x_;
    MatrixXX P_;
    rclcpp::Time tic;

private:
    std::priority_queue<std::shared_ptr<MeasurementInterface>, std::vector<std::shared_ptr<MeasurementInterface>>, std::greater<>> queue_;
    std::vector<std::shared_ptr<ModelProcessInterface>> models_process_;
    std::vector<std::shared_ptr<ModelMeasurementInterface>> models_measurement_;
};

} // end namespace filters_base

#include "filters_base/base_impl.hpp"

#endif // _FILTERS_BASE_HPP
