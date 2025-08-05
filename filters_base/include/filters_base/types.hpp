#ifndef _FILTERS_BASE_TYPES_HPP
#define _FILTERS_BASE_TYPES_HPP

#include <Eigen/Dense>

namespace filters_base
{

template<int NX, int NU, int NW>
struct TypesProcess
{
    static constexpr int nx = NX;
    static constexpr int nu = NU;
    static constexpr int nw = NW;

    using VectorX = Eigen::Matrix<double, NX, 1>;
    using VectorU = Eigen::Matrix<double, NU, 1>;
    using VectorW = Eigen::Matrix<double, NW, 1>;
    
    using MatrixXX = Eigen::Matrix<double, NX, NX>;
    using MatrixWW = Eigen::Matrix<double, NW, NW>;
};

template<typename ProcessT, int NZ>
struct TypesMeasurement
{
    static constexpr int nz = NZ;
    
    using VectorX = typename ProcessT::VectorX;
    using VectorZ = Eigen::Matrix<double, NZ, 1>;

    using MatrixZX = Eigen::Matrix<double, NZ, ProcessT::nx>;
    using MatrixZZ = Eigen::Matrix<double, NZ, NZ>;
};

} // end namespace filters_base

#endif // _FILTERS_BASE_TYPES_HPP
