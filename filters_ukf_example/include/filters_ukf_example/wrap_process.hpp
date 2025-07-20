#ifndef _FILTERS_UKF_EXAMPLE_WRAP_PROCESS_HPP
#define _FILTERS_UKF_EXAMPLE_WRAP_PROCESS_HPP

#include "filters_ukf_core/core.hpp"
#include "filters_ukf_example/parameters.hpp"

namespace filters_ukf_example
{

template<int N_X, int N_U, int N_W>
class F : public filters_ukf_core::ModelProcess<N_X, N_U, N_W> {
public:
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorU = Eigen::Matrix<double, N_U, 1>;
    using VectorW = Eigen::Matrix<double, N_W, 1>;

    VectorX f(const VectorX& x, const VectorU& u, const VectorW& w, double t) const override {
        VectorX dx;

        (void) u;
        (void) t;

        dx(0) = x(1);
        dx(1) = -14.0 * abs(x(1)) * x(1) + w(0);

        return dx;
    }
};

} // namespace filters_ukf_example

#endif // _FILTERS_UKF_EXAMPLE_WRAP_PROCESS_HPP
