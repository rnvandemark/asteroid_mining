#pragma once

#include "asteroid_mining/memento_i.hpp"
#include "asteroid_mining/siphon/carrier.hpp"

namespace asteroid_mining {

class SiphonMemento : public SiphonCarrier, public MementoI<SiphonCarrier>
{
public:
    using SiphonCarrier::SiphonCarrier;

    virtual void set_state(const SiphonCarrier& carrier) override;
};

}
