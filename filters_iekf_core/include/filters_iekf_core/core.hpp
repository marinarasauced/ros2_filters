#ifndef _FILTERS_IEKF_CORE_HPP
#define _FILTERS_IEKF_CORE_HPP

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "filters_iekf_core/lie_group_traits.hpp"
#include "filters_iekf_core/model_measurement.hpp"
#include "filters_iekf_core/model_process.hpp"

namespace filters_iekf_core
{

template<int N_X, int N_U, int N_W>
class IEKFCore
{
public:
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorU = Eigen::Matrix<double, N_U, 1>;
    using VectorW = Eigen::Matrix<double, N_W, 1>;
    using MatrixX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixW = Eigen::Matrix<double, N_W, N_W>;

    IEKFCore(
        const std::shared_ptr<ModelProcess<N_X, N_U, N_W>>& model_process,
        const VectorX& x0,
        const MatrixX& P0,
        const MatrixW& Q0
    );

    VectorX computeRK4Step(const VectorX& x, const VectorU& u, const VectorW& w, double t, double dt);
    void predictMP(const VectorU& u, double t, double dt);

    template<int N_Z>
    void updateMM(std::shared_ptr<ModelMeasurement<N_X, N_Z>> model_measurement, const Eigen::Matrix<double, N_Z, 1>& z);

    VectorX getX() const;
    MatrixX getP() const;

private:
    mutable std::mutex mutex_;

    VectorX x_;
    MatrixX P_;
    MatrixW Q_;

    std::shared_ptr<ModelProcess<N_X, N_U, N_W>> model_process_;
};

} // namespace filters_iekf_core

#include "filters_iekf_core/core_impl.hpp"

#endif // _FILTERS_IEKF_CORE_HPP
