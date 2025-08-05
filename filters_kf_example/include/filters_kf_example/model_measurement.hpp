#ifndef _FILTERS_KF_EXAMPLE_MODEL_MEASUREMENT_HPP
#define _FILTERS_KF_EXAMPLE_MODEL_MEASUREMENT_HPP

#include "filters_base/model_measurement.hpp"
#include "filters_base/types.hpp"
#include "filters_kf_example/model_process.hpp"

namespace filters_kf_example
{

struct KFExampleMeasurementT : public filters_base::TypesMeasurement<KFExampleProcessT, 1>
{
};


class KFExampleMeasurement : public filters_base::ModelMeasurement<KFExampleMeasurementT>
{
public:
    using VectorX = KFExampleProcessT::VectorX;
    using VectorZ = KFExampleMeasurementT::VectorZ;
    using MatrixZX = KFExampleMeasurementT::MatrixZX;
    using MatrixZZ = KFExampleMeasurementT::MatrixZZ;

    VectorZ h(const VectorX& x) const override
    {
        VectorZ z;
        z(0) = x(1);
        return z;
    }

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

#endif // _FILTERS_KF_EXAMPLE_MODEL_MEASUREMENT_HPP
