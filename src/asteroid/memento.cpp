#include "asteroid_mining/asteroid/memento.hpp"

namespace asteroid_mining {

void AsteroidMemento::set_state(const AsteroidCarrier& carrier)
{
    alpha = carrier.alpha;
    beta = carrier.beta;
    gamma = carrier.gamma;
    density = carrier.density;
    angular_velocity = carrier.angular_velocity;
    omega = carrier.omega;
    rotation = carrier.rotation;
}

}
