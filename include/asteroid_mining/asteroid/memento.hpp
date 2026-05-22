#pragma once

#include "asteroid_mining/asteroid/carrier.hpp"
#include "asteroid_mining/memento_i.hpp"

namespace asteroid_mining {

class AsteroidMemento : public AsteroidCarrier, public MementoI<AsteroidCarrier>
{
public:
    using AsteroidCarrier::AsteroidCarrier;

    virtual void set_state(const AsteroidCarrier& carrier) override;
};

}
