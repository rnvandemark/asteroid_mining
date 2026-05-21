#include "asteroid_mining/dimensions_scaler.hpp"

#include <cmath>

namespace asteroid_mining {

DimensionsScaler::ScaleFactor::ScaleFactor(const DimensionsScaler::ScaleFactor::DimensionType& d_, const double n_):
    d(d_), n(n_)
{
}

DimensionsScaler::ScaleOpChain::ScaleOpChain(const DimensionsScaler::ScaleOpChain::ChainT& chain_):
    chain(chain_)
{
}

DimensionsScaler::ScaleOpChain DimensionsScaler::ScaleOpChain::operator*(const DimensionsScaler::ScaleFactor& rhs) const
{
    return appended(rhs, DimensionsScaler::ScaleOpChain::Op::MULT);
}
DimensionsScaler::ScaleOpChain DimensionsScaler::ScaleOpChain::operator/(const DimensionsScaler::ScaleFactor& rhs) const
{
    return appended(rhs, DimensionsScaler::ScaleOpChain::Op::DIV);
}

DimensionsScaler::ScaleOpChain DimensionsScaler::ScaleOpChain::appended(
    const DimensionsScaler::ScaleFactor& scale_factor,
    const DimensionsScaler::ScaleOpChain::Op& op
) const
{
    DimensionsScaler::ScaleOpChain::ChainT new_chain = chain;
    new_chain.push_back(std::make_pair(scale_factor, op));
    return DimensionsScaler::ScaleOpChain(new_chain);
}

DimensionsScaler::DimensionsScaler(
    const double distance_scale_,
    const double mass_scale_,
    const double time_scale_
):
    distance_scale(distance_scale_),
    mass_scale(mass_scale_),
    time_scale(time_scale_)
{
}

double DimensionsScaler::get_dimensionless(const double dimensioned, const DimensionsScaler::ScaleOpChain& chain) const
{
    return dimensioned / apply_chain(chain);
}
double DimensionsScaler::get_dimensioned(const double dimensionless, const DimensionsScaler::ScaleOpChain& chain) const
{
    return dimensionless * apply_chain(chain);
}

double DimensionsScaler::apply_chain(const DimensionsScaler::ScaleOpChain& chain) const
{
    double s = 1.0;
    for (const auto& scale_op : chain.chain)
    {
        const double scale = (
            (scale_op.first.d == DimensionsScaler::ScaleFactor::DimensionType::DISTANCE) ? distance_scale : (
                (scale_op.first.d == DimensionsScaler::ScaleFactor::DimensionType::MASS) ? mass_scale : (
                    (scale_op.first.d == DimensionsScaler::ScaleFactor::DimensionType::TIME) ? time_scale : 1.0
                )
            )
        );
        if (scale_op.second == DimensionsScaler::ScaleOpChain::Op::MULT)
        {
            s *= std::pow(scale, scale_op.first.n);
        }
        else if (scale_op.second == DimensionsScaler::ScaleOpChain::Op::DIV)
        {
            s /= std::pow(scale, scale_op.first.n);
        }
    }
    return s;
}

}
