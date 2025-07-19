#ifndef _FILTERS_UKF_CORE_HPP
#define _FILTERS_UKF_CORE_HPP

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "filters_ukf_core/model_measurement.hpp"
#include "filters_ukf_core/model_process.hpp"

namespace filters_ukf_core
{

template<int N_X, int N_U, int N_W>
class UKFCore
{
public:
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using MatrixX = Eigen::Matrix<double, N_X, N_X>;
    using VectorW = Eigen::Matrix<double, N_W, 1>;
    using MatrixW = Eigen::Matrix<double, N_W, N_W>;

    using VectorU = Eigen::Matrix<double, N_U, 1>;

    using model_process = ModelProcess<N_X, N_U, N_W>;

    UKFCore(
        std::shared_ptr<model_process> model_process,
        const MatrixX& P,
        const VectorW& w_mean,
        const Eigen::Matrix<double, N_W, N_W>& Q
    );

    VectorX rk4_step(const VectorX& x, const VectorU& u, const VectorW& w, double dt);
    void predict(double dt);

    template<int N_Z>
    std::shared_ptr<ModelMeasurement<N_X, N_Z>> add_model_measurement(
        std::shared_ptr<ModelMeasurement<N_X, N_Z>> model_measurement
    );

    template<int N_Z>
    void update(
        const std::shared_ptr<ModelMeasurement<N_X, N_Z>>& model_measurement,
        const Eigen::Matrix<double, N_Z, 1>& z
    );


    VectorX get_x() const;
    MatrixX get_P() const;

    Eigen::VectorXd get_y() const;
    Eigen::MatrixXd get_S() const;

    void set_u(const VectorU& u);

private:
    static constexpr int N_AUG = N_X + N_W;
    static constexpr int N_SIGMA = 2 * N_AUG + 1;

    VectorX x_;
    MatrixX P_;
    VectorW w_mean_;
    MatrixW Q_;

    VectorU u_;

    static constexpr double alpha_ = 1e-3;
    static constexpr double beta_ = 2.0;
    static constexpr double kappa_ = 0.0;
    double lambda_;
    double gamma_;

    using MatrixSigma = Eigen::Matrix<double, N_AUG, N_SIGMA>;

    std::shared_ptr<model_process> model_process_;

    std::vector<double> weights_mean_;
    std::vector<double> weights_cov_;

    mutable std::mutex mutex_;

    struct InterfaceWrapperMM {
        virtual ~InterfaceWrapperMM() = default;
    };

    template<int N_Z>
    struct WrapperMM : InterfaceWrapperMM {
        std::shared_ptr<ModelMeasurement<N_X, N_Z>> model_measurement;
        WrapperMM(std::shared_ptr<ModelMeasurement<N_X, N_Z>> m) : model_measurement(m) {}
    };

    std::vector<std::shared_ptr<InterfaceWrapperMM>> models_measurement_;
    Eigen::VectorXd y_;
    Eigen::MatrixXd S_;

    void _compute_weights();
    Eigen::Matrix<double, N_X + N_W, 2 * (N_X + N_W) + 1> _generate_sigma_points(const VectorX& x, const MatrixX& P);
};

} // namespace filters_ukf_core

#include "filters_ukf_core/core_impl.hpp"

#endif // _FILTERS_UKF_CORE_HPP
