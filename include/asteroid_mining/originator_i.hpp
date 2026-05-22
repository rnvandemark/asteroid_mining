#pragma once

namespace asteroid_mining {

template<typename CarrierT>
class OriginatorI
{
public:
    virtual const CarrierT& get_state() const = 0;
};

}
