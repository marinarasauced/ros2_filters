#ifndef _FILTERS_BASE_MODEL_PROCESS_HPP
#define _FILTERS_BASE_MODEL_PROCESS_HPP

#include "filters_base/types.hpp"

namespace filters_base
{

class ModelProcessInterface
{
public:
    virtual ~ModelProcessInterface() = default;
};


template<typename ProcessT>
class ModelProcess : public ModelProcessInterface
{
public:
    using VectorX = typename ProcessT::VectorX;
    using VectorU = typename ProcessT::VectorU;
    using VectorW = typename ProcessT::VectorW;
    using MatrixXX = typename ProcessT::MatrixXX;
    using MatrixWW = typename ProcessT::MatrixWW;

    virtual ~ModelProcess() = default;
    
    virtual VectorX f(const VectorX& x, const VectorU& u, const VectorW& w, double t, double dt) const { throw std::runtime_error("f() not implemented"); };
    virtual MatrixXX F(const VectorX& x, double t, double dt) const { throw std::runtime_error("F() not implemented"); };
    virtual MatrixXX W(const VectorW& w, double t, double dt) const { throw std::runtime_error("W() not implemented"); };

    virtual MatrixWW Q(double t, double dt) const { throw std::runtime_error("Q() not implemented"); };
};

} // end namespace filters_base

#endif // _FILTERS_BASE_MODEL_PROCESS_HPP
