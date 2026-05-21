#pragma once

#include <utility>
#include <vector>

namespace asteroid_mining {

class DimensionsScaler
{
public:
    class ScaleFactor
    {
    public:
        enum class DimensionType { DISTANCE, MASS, TIME };
        ScaleFactor(const DimensionType& d_, const double n_ = 1);
        const DimensionType d;
        const double n;
    };

    class ScaleOpChain
    {
    public:
        enum class Op { MULT, DIV };
        using ChainT = std::vector<std::pair<DimensionsScaler::ScaleFactor, DimensionsScaler::ScaleOpChain::Op>>;
        ScaleOpChain(const ChainT& chain_ = ChainT());

        ScaleOpChain operator*(const ScaleFactor& rhs) const;
        ScaleOpChain operator/(const ScaleFactor& rhs) const;

        const ChainT chain;

    protected:
        ScaleOpChain appended(
            const ScaleFactor& scale_factor,
            const DimensionsScaler::ScaleOpChain::Op& op
        ) const;
    };

    DimensionsScaler(
        const double distance_scale_,
        const double mass_scale_,
        const double time_scale_
    );

    double get_dimensionless(const double dimensioned, const DimensionsScaler::ScaleOpChain& chain) const;
    double get_dimensioned(const double dimensionless, const DimensionsScaler::ScaleOpChain& chain) const;

protected:
    double apply_chain(const DimensionsScaler::ScaleOpChain& chain) const;

    const double distance_scale;
    const double mass_scale;
    const double time_scale;
};

}
