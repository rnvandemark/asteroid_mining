#pragma once

namespace asteroid_mining {

template<typename CarrierT>
class MementoI
{
public:
    virtual void set_state(const CarrierT& carrier) = 0;
};

}
