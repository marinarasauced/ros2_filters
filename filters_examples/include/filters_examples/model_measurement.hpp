#ifndef FILTERS_EXAMPLES_MODEL_MEASUREMENT_HPP
#define FILTERS_EXAMPLES_MODEL_MEASUREMENT_HPP

#include "filters_base/model_measurement.hpp"
#include "filters_base/types.hpp"
#include "filters_kf_example/model_process.hpp"

namespace filters_kf_example
{

struct MeasurementT : public filters_base::TypesMeasurement<ProcessT, 1>
{
};


class ModelMeasurement : public filters_base::ModelMeasurement<MeasurementT>
{
public:
    using VectorX = ProcessT::VectorX;
    using VectorZ = MeasurementT::VectorZ;
    using MatrixZX = MeasurementT::MatrixZX;
    using MatrixZZ = MeasurementT::MatrixZZ;

    MatrixZX H(const VectorX& x) const override
    {
        MatrixZX H;
        H(0, 0) = 0.0;
        H(0, 1) = 1.0;
        return H;
    }

    MatrixZZ R() const override
    {
        MatrixZZ R;
        R(0, 0) = 0.1;
        return R;
    }
};

} // end namespace filters_kf_core

#endif // FILTERS_EXAMPLES_MODEL_MEASUREMENT_HPP