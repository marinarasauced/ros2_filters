#ifndef _FILTERS_BASE_MODEL_PROCESS_HPP
#define _FILTERS_BASE_MODEL_PROCESS_HPP

#include "filters_base/types.hpp"

namespace filters_base
{

template<typename ProcessT>
class ModelProcess
{
public:
    using VectorX = typename ProcessT::VectorX;
    using VectorU = typename ProcessT::VectorU;
    using VectorW = typename ProcessT::VectorW;
    using MatrixXX = typename ProcessT::MatrixXX;
    using MatrixWW = typename ProcessT::MatrixWW;

    virtual ~ModelProcess() = default;
    
    virtual VectorX f(const VectorX& x, const VectorU& u, const VectorW& w, double t, double dt) const = 0;
    virtual MatrixXX F(const VectorX& x, const VectorU& u, const VectorW& w, double t, double dt) const = 0;

    virtual MatrixXX A(const VectorX& x, double t, double dt) const = 0;
    virtual MatrixXX G(const VectorW& w, double t, double dt) const = 0;

    virtual MatrixWW Q(double t, double dt) const = 0;
};

} // end namespace filters_base

#endif // _FILTERS_BASE_MODEL_PROCESS_HPP
