#include "asteroid_mining/released_payload/memento.hpp"

namespace asteroid_mining {

void ReleasedPayloadMemento::set_state(const ReleasedPayloadCarrier& carrier)
{
    active = carrier.active;
    mass = carrier.mass;
    position = carrier.position;
    velocity = carrier.velocity;
    acceleration = carrier.acceleration;
}

}
