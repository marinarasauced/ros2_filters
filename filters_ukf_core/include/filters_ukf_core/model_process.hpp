#ifndef _FILTERS_UKF_MODEL_PROCESS_HPP
#define _FILTERS_UKF_MODEL_PROCESS_HPP

#include <Eigen/Dense>
// #include <Eigen/Geometry>

namespace filters_ukf_core
{

template<int N_X, int N_U, int N_W>
class ModelProcess
{
public:
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorU = Eigen::Matrix<double, N_U, 1>;
    using VectorW = Eigen::Matrix<double, N_W, 1>;

    virtual ~ModelProcess() = default;

    virtual VectorX f(
        const VectorX& x,
        const VectorU& u,
        const VectorW& w,
        double t
    ) const = 0;
};

} // namespace filters_ukf_core

#endif // _FILTERS_UKF_MODEL_PROCESS_HPP
