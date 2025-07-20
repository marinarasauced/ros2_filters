#ifndef _FILTERS_UKF_EXAMPLE_WRAP_MEASUREMENT_HPP
#define _FILTERS_UKF_EXAMPLE_WRAP_MEASUREMENT_HPP

#include "filters_ukf_core/core.hpp"
#include "filters_ukf_example/parameters.hpp"

namespace filters_ukf_example
{

template <int N_X, int N_Z>
class H2 : public filters_ukf_core::ModelMeasurement<N_X, 1>
{
public:
    using VectorX = typename filters_ukf_core::ModelMeasurement<N_X, 1>::VectorX;
    using VectorZ = typename filters_ukf_core::ModelMeasurement<N_X, 1>::VectorZ;
    using MatrixZ = typename filters_ukf_core::ModelMeasurement<N_X, 1>::MatrixZ;

    H2(const BlueROV2Parameters& params, double sigma)
        : R_(MatrixZ::Identity() * sigma * sigma), params_(params) {}

    VectorZ h(const VectorX& x) const override {
        VectorZ z;
        z(0) = params_.z2.pressure_atm + params_.z2.rho * params_.gravity * x(0);

        return z;
    }

    MatrixZ R() const override {
        return R_;
    }

private:
    MatrixZ R_;
    BlueROV2Parameters params_;
};

} // namespace filters_ukf_example

#endif // _FILTERS_UKF_EXAMPLE_WRAP_MEASUREMENT_HPP