#ifndef _FILTERS_IEKF_MODEL_MEASUREMENT_HPP
#define _FILTERS_IEKF_MODEL_MEASUREMENT_HPP

#include <Eigen/Dense>

namespace filters_iekf_core
{

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

    VectorZ getInnovation() const { return y_; }
    void setInnovation(const VectorZ& y) { y_ = y; }

    virtual Eigen::Matrix<double, N_Z, N_X> jacobianH(const Eigen::Matrix<double, N_X, 1>& x) const = 0;

private:
    VectorZ y_;
};

} // namespace filters_iekf_core

#endif // _FILTERS_IEKF_MODEL_MEASUREMENT_HPP