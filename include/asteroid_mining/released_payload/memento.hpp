#pragma once

#include "asteroid_mining/memento_i.hpp"
#include "asteroid_mining/released_payload/carrier.hpp"

namespace asteroid_mining {

class ReleasedPayloadMemento : public ReleasedPayloadCarrier, public MementoI<ReleasedPayloadCarrier>
{
public:
    using ReleasedPayloadCarrier::ReleasedPayloadCarrier;

    virtual void set_state(const ReleasedPayloadCarrier& carrier) override;
};

}
