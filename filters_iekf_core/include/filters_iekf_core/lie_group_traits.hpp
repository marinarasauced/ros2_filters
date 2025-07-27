#ifndef _FILTERS_IEKF_LIE_GROUP_TRAITS_HPP
#define _FILTERS_IEKF_LIE_GROUP_TRAITS_HPP

namespace filters_iekf_core
{

template<int N_X>
struct LieGroupTraits {
    static Eigen::Matrix<double, N_X, 1> log(const Eigen::Matrix<double, N_X, 1>& x);
    static Eigen::Matrix<double, N_X, 1> exp(const Eigen::Matrix<double, N_X, 1>& dx);
    static Eigen::Matrix<double, N_X, 1> retract(const Eigen::Matrix<double, N_X, 1>& x, const Eigen::Matrix<double, N_X, 1>& dx);
    static Eigen::Matrix<double, N_X, 1> inverse(const Eigen::Matrix<double, N_X, 1>& x);
    static Eigen::Matrix<double, N_X, N_X> adjoint(const Eigen::Matrix<double, N_X, 1>& x);
};

} // namespace filters_iekf_lie_group_traits

#endif // _FILTERS_IEKF_LIE_GROUP_TRAITS_HPP
