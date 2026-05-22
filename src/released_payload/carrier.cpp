#include "asteroid_mining/released_payload/carrier.hpp"

namespace asteroid_mining {

ReleasedPayloadCarrier::ReleasedPayloadCarrier():
    ReleasedPayloadCarrier(
        false,
        0.0,
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    )
{
}

ReleasedPayloadCarrier::ReleasedPayloadCarrier(
    const bool active_,
    const double mass_,
    const Eigen::Vector3d& position_,
    const Eigen::Vector3d& velocity_,
    const Eigen::Vector3d& acceleration_
):
    active(active_),
    mass(mass_),
    position(position_),
    velocity(velocity_),
    acceleration(acceleration_)
{
}

bool ReleasedPayloadCarrier::is_active() const
{
    return active;
}

double ReleasedPayloadCarrier::get_mass() const
{
    return mass;
}
const Eigen::Vector3d& ReleasedPayloadCarrier::get_position() const
{
    return position;
}
const Eigen::Vector3d& ReleasedPayloadCarrier::get_velocity() const
{
    return velocity;
}
const Eigen::Vector3d& ReleasedPayloadCarrier::get_acceleration() const
{
    return acceleration;
}

}
