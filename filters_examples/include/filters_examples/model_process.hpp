#ifndef _FILTERS_EXAMPLES_MODEL_PROCESS_HPP
#define _FILTERS_EXAMPLES_MODEL_PROCESS_HPP

#include "filters_base/model_process.hpp"

namespace filters_examples
{

struct ProcessT : public filters_base::TypesProcess<2, 1, 2>
{
};


class ModelProcess : public filters_base::ModelProcess<ProcessT>
{
public:
    using VectorX = ProcessT::VectorX;
    using VectorU = ProcessT::VectorU;
    using VectorW = ProcessT::VectorW;
    using MatrixXX = ProcessT::MatrixXX;
    using MatrixWW = ProcessT::MatrixWW;

    VectorX f(const VectorX& x, const VectorU& u, const VectorW& w, double t, double dt) const override
    {
        VectorX dx;
        dx(0) = x(1);
        dx(1) = u(0) + w(0);
        return dx;
    }

    MatrixXX A(const VectorX& x, double t, double dt) const override
    {
        MatrixXX A;
        A(0, 0) = 1.0;
        A(0, 1) = dt;
        A(1, 0) = 0.0;
        A(1, 1) = 1.0;
        return A;
    }

    MatrixXX G(const VectorW& w, double t, double dt) const override
    {
        MatrixXX G;
        G(0, 0) = 1.0;
        G(0, 1) = 0.0;
        G(1, 0) = 0.0;
        G(1, 1) = 1.0;
        return G;
    }

    MatrixWW Q(double t, double dt) const override
    {
        return MatrixWW::Identity() * 0.01;
    }
};

} // end namespace filters_examples

#endif // _FILTERS_EXAMPLES_MODEL_PROCESS_HPP