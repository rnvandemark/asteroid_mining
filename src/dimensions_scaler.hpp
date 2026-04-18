#pragma once

#include <cmath>
#include <utility>
#include <vector>

namespace am {

class DimensionsScaler
{
public:
    class ScaleFactor
    {
    public:
        enum class DimensionType { DISTANCE, MASS, TIME };
        ScaleFactor(const DimensionType& d_, const double n_ = 1) :
            d(d_), n(n_)
        {
        }
        const DimensionType d;
        const double n;
    };

    class ScaleOpChain
    {
    public:
        enum class Op { MULT, DIV };
        using ChainT = std::vector<std::pair<DimensionsScaler::ScaleFactor, DimensionsScaler::ScaleOpChain::Op>>;
        ScaleOpChain(const ChainT& chain_ = ChainT()) :
            chain(chain_)
        {
        }

        ScaleOpChain operator*(const ScaleFactor& rhs) const
        {
            return appended(rhs, DimensionsScaler::ScaleOpChain::Op::MULT);
        }
        ScaleOpChain operator/(const ScaleFactor& rhs) const
        {
            return appended(rhs, DimensionsScaler::ScaleOpChain::Op::DIV);
        }

        const ChainT chain;

    protected:
        ScaleOpChain appended(const ScaleFactor& scale_factor, const DimensionsScaler::ScaleOpChain::Op& op) const
        {
            ChainT new_chain = chain;
            new_chain.push_back(std::make_pair(scale_factor, op));
            return ScaleOpChain(new_chain);
        }
    };

    DimensionsScaler(
        const double distance_scale_,
        const double mass_scale_,
        const double time_scale_
    ):
        distance_scale(distance_scale_),
        mass_scale(mass_scale_),
        time_scale(time_scale_)
    {
    }

    double get_dimensionless(const double dimensioned, const DimensionsScaler::ScaleOpChain& chain) const
    {
        return dimensioned / apply_chain(chain);
    }
    double get_dimensioned(const double dimensionless, const DimensionsScaler::ScaleOpChain& chain) const
    {
        return dimensionless * apply_chain(chain);
    }

protected:
    double apply_chain(const DimensionsScaler::ScaleOpChain& chain) const
    {
        double s = 1.0;
        for (const auto& scale_op : chain.chain)
        {
            const double scale = (
                (scale_op.first.d == ScaleFactor::DimensionType::DISTANCE) ? distance_scale : (
                    (scale_op.first.d == ScaleFactor::DimensionType::MASS) ? mass_scale : (
                        (scale_op.first.d == ScaleFactor::DimensionType::TIME) ? time_scale : 1.0
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

    const double distance_scale;
    const double mass_scale;
    const double time_scale;
};

}
