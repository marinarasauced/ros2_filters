#ifndef _FILTERS_KF_CORE_HPP
#define _FILTERS_KF_CORE_HPP

#include "filters_base/base.hpp"

namespace filters_kf_core
{

template<typename ProcessT>
class FilterKF : public filters_base::Filter<ProcessT>
{
public:
    using VectorX = typename ProcessT::VectorX;
    using VectorU = typename ProcessT::VectorU;
    using VectorW = typename ProcessT::VectorW;
    using MatrixXX = typename ProcessT::MatrixXX;
    using MatrixWW = typename ProcessT::MatrixWW;

    FilterKF(const VectorX& x0, const MatrixXX& P0, const rclcpp::Time& tic);

    void predict(const std::shared_ptr<filters_base::ModelProcess<ProcessT>>& mp, const VectorU& u, double t, double dt) override;

    template<typename MeasurementT>
    void update(const std::shared_ptr<filters_base::ModelMeasurement<MeasurementT>>& mm, const typename MeasurementT::VectorZ& z, double t);
};

} // end namespace filters_kf_core

#include "filters_kf_core/core_impl.hpp"

#endif // _FILTERS_KF_CORE_HPP
