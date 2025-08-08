#ifndef _FILTERS_CORE_KF_HPP
#define _FILTERS_CORE_KF_HPP

#include "filters_base/base.hpp"

namespace filters_core_kf
{

template<typename ProcessT>
class FilterCore : public filters_base::FilterBase<ProcessT>
{
public:
    using VectorX = typename ProcessT::VectorX;
    using VectorU = typename ProcessT::VectorU;
    using VectorW = typename ProcessT::VectorW;
    using MatrixXX = typename ProcessT::MatrixXX;
    using MatrixWW = typename ProcessT::MatrixWW;

    FilterCore(const VectorX& x0, const MatrixXX& P0, const rclcpp::Time& tic);

    void predict(const std::shared_ptr<filters_base::ModelProcess<ProcessT>>& mp, const VectorU& u, double t, double dt) override;

    template<typename MeasurementT>
    void update(const std::shared_ptr<filters_base::ModelMeasurement<MeasurementT>>& mm, const typename MeasurementT::VectorZ& z, double t);
};

} // end namespace filters_core_kf

#include "filters_core_kf/core_impl.hpp"

#endif // _FILTERS_CORE_KF_HPP
