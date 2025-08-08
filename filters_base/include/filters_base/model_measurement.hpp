#ifndef _FILTERS_BASE_MODEL_MEASUREMENT_HPP
#define _FILTERS_BASE_MODEL_MEASUREMENT_HPP

#include "filters_base/types.hpp"

namespace filters_base
{

class ModelMeasurementInterface
{
public:
    virtual ~ModelMeasurementInterface() = default;
};


template<typename MeasurementT>
class ModelMeasurement : public ModelMeasurementInterface
{
public:
    using VectorX = typename MeasurementT::VectorX;
    using VectorZ = typename MeasurementT::VectorZ;
    using MatrixZX = typename MeasurementT::MatrixZX;
    using MatrixZZ = typename MeasurementT::MatrixZZ;

    virtual ~ModelMeasurement() = default;

    virtual VectorZ h(const VectorX& x) const { throw std::runtime_error("h() not implemented"); };
    virtual MatrixZX H(const VectorX& x) const { throw std::runtime_error("H() not implemented"); };

    virtual MatrixZZ R() const { throw std::runtime_error("R() not implemented"); };
};

} // end namespace filters_base

#endif // _FILTERS_BASE_MODEL_MEASUREMENT_HPP
