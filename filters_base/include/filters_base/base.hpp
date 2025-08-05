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

template<typename ProcessT, typename MeasurementT>
class FilterInterface
{
public:
    using VectorX = typename ProcessT::VectorX;
    using VectorU = typename ProcessT::VectorU;
    using VectorW = typename ProcessT::VectorW;
    using MatrixXX = typename ProcessT::MatrixXX;

    virtual ~FilterInterface() = default;

    virtual void predict(const ModelProcess<ProcessT>& mp, const ProcessT::VectorU& u, const double t, const double dt) = 0;
    virtual void update(const ModelMeasurement<MeasurementT>& mm, const MeasurementT::VectorZ& z, const double t) = 0;
};


template<typename ProcessT, typename MeasurementT>
class Filter : public FilterInterface<ProcessT, MeasurementT>
{
public:
    using VectorX = typename ProcessT::VectorX;
    using VectorU = typename ProcessT::VectorU;
    using VectorW = typename ProcessT::VectorW;
    using MatrixXX = typename ProcessT::MatrixXX;

    Filter(const VectorX& x0, const MatrixXX& P0, const rclcpp::Time& tic);

    VectorX rk4step(const std::shared_ptr<ModelProcess<ProcessT>> model_process, const VectorX& x, const VectorU& u, const VectorW& w, double t, double dt) const;

    void enqueue(const std::shared_ptr<MeasurementInterface>& measurement);
    void dequeue(const rclcpp::Time toc);

protected:
    VectorX x_;
    MatrixXX P_;
    rclcpp::Time tic;

private:
    std::priority_queue<std::shared_ptr<MeasurementInterface>, std::vector<std::shared_ptr<MeasurementInterface>>, std::greater<>> queue_;

    std::shared_ptr<ModelProcess<ProcessT>> model_process_;
    std::vector<std::shared_ptr<ModelMeasurement<MeasurementT>>> models_measurement_;
};

} // end namespace filters_base

#include "filters_base/base_impl.hpp"

#endif // _FILTERS_BASE_HPP
