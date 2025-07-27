#ifndef _FILTERS_IEKF_MODEL_PROCESS_HPP
#define _FILTERS_IEKF_MODEL_PROCESS_HPP

#include <Eigen/Dense>

namespace filters_iekf_core
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

    virtual Eigen::Matrix<double, N_X, N_X> jacobianX(const VectorX& x, const VectorU& u, const VectorW& w, double t) const = 0;
    virtual Eigen::Matrix<double, N_X, N_W> jacobianW(const VectorX& x, const VectorU& u, const VectorW& w, double t) const = 0;
};

} // namespace filters_iekf_core

#endif // _FILTERS_IEKF_MODEL_PROCESS_HPP
