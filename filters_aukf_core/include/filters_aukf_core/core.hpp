#ifndef _FILTERS_AUKF_CORE_HPP
#define _FILTERS_AUKF_CORE_HPP

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "filters_aukf_core/model_measurement.hpp"
#include "filters_aukf_core/model_process.hpp"

namespace filters_aukf_core
{

template<int N_X, int N_U, int N_W>
class AUKFCore
{
public:
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorU = Eigen::Matrix<double, N_U, 1>;
    using VectorW = Eigen::Matrix<double, N_W, 1>;
    using MatrixX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixW = Eigen::Matrix<double, N_W, N_W>;

    AUKFCore(
        const std::shared_ptr<ModelProcess<N_X, N_U, N_W>>& model_process,
        const VectorX& x0,
        const MatrixX& P0,
        const MatrixW& Q0,
        double alpha = 1e-3,
        double beta = 2.0,
        double kappa = 0.0
    );

    VectorX computeRK4Step(const VectorX& x, const VectorU& u, const VectorW& w, double t, double dt);
    void predictMP(const VectorU& u, double t, double dt);

    template<int N_Z>
    void addMeasurementModel(std::shared_ptr<ModelMeasurement<N_X, N_Z>> model_measurement);

    template<int N_Z>
    void updateMM(std::shared_ptr<ModelMeasurement<N_X, N_Z>> model_measurement, Eigen::Matrix<double, N_Z, 1>& z);


    VectorX getX() const;
    MatrixX getP() const;
    MatrixW getQ() const;

private:
    mutable std::mutex mutex_;

    static constexpr int N_AUG = N_X + N_W;
    static constexpr int N_SIGMA = 2 * N_AUG + 1;

    VectorX x_;
    MatrixX P_;
    MatrixW Q_;

    double alpha_;
    double beta_ ;
    double kappa_;
    double lambda_;
    double gamma_;

    std::shared_ptr<ModelProcess<N_X, N_U, N_W>> model_process_;

    Eigen::VectorXd weights_mean_;
    Eigen::VectorXd weights_cov_;

    struct WrapperInterface {
        virtual ~WrapperInterface() = default;
    };

    template<int N_Z>
    struct Wrapper : WrapperInterface {
        std::shared_ptr<ModelMeasurement<N_X, N_Z>> model_measurement;
        Wrapper(std::shared_ptr<ModelMeasurement<N_X, N_Z>> model_measurement) : model_measurement(model_measurement) {}
    };

    std::vector<std::shared_ptr<WrapperInterface>> models_measurement_;

    void computeWeights();
    void computeAugmentedSigmaPoints(Eigen::Matrix<double, N_AUG, N_SIGMA>& x_sigma);
};

} // namespace filters_aukf_core

#include "filters_aukf_core/core_impl.hpp"

#endif // _FILTERS_AUKF_CORE_HPP
