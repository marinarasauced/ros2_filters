#ifndef _FILTERS_UKF_MODEL_MEASUREMENT_HPP
#define _FILTERS_UKF_MODEL_MEASUREMENT_HPP

#include <Eigen/Dense>

namespace filters_ukf_core
{

template<int N_X, int N_U, int N_W>
class UKFCore; // forward declaration

template<int N_X, int N_Z>
class ModelMeasurement
{
public:
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorZ = Eigen::Matrix<double, N_Z, 1>;
    using MatrixZ = Eigen::Matrix<double, N_Z, N_Z>;

    virtual ~ModelMeasurement() = default;

    virtual VectorZ h(const VectorX& x) const = 0;
    virtual MatrixZ R() const = 0;
};

} // namespace filters_ukf_core

#endif // _FILTERS_UKF_MODEL_MEASUREMENT_HPP