#include "asteroid_mining/released_payload.hpp"

#include <iostream>

namespace asteroid_mining {

ReleasedPayload::ReleasedPayload(const Asteroid& asteroid_):
    asteroid(asteroid_),
    active(false),
    mass(0.0),
    position(),
    velocity(),
    acceleration()
{
}

bool ReleasedPayload::is_active() const
{
    return active;
}

const Eigen::Vector3d& ReleasedPayload::get_position() const
{
    return position;
}
const Eigen::Vector3d& ReleasedPayload::get_velocity() const
{
    return velocity;
}

void ReleasedPayload::release(
    const double mass_,
    const Eigen::Vector3d& position_,
    const Eigen::Vector3d& velocity_,
    const Eigen::Vector3d& acceleration_
)
{
    mass = mass_;
    position = position_;
    velocity = velocity_;
    acceleration = acceleration_;
    active = true;
}

void ReleasedPayload::progress_over(const double dt)
{
    if (active)
    {
        position.x() += (velocity.x() * dt) + (0.5 * acceleration.x() * dt * dt);
        position.y() += (velocity.y() * dt) + (0.5 * acceleration.y() * dt * dt);
        position.z() += (velocity.z() * dt) + (0.5 * acceleration.z() * dt * dt);

        velocity.x() += (acceleration.x() * dt);
        velocity.y() += (acceleration.y() * dt);
        velocity.z() += (acceleration.z() * dt);

        double effective_force_magnitude = 0;
        const auto effective_force = asteroid.calculate_cartesian_effective_force_at(position, effective_force_magnitude);

        acceleration.x() = effective_force.x() + (2 * velocity.y());
        acceleration.y() = effective_force.y() - (2 * velocity.x());
        acceleration.z() = effective_force.z();

        if (asteroid.is_point_within(position))
        {
            std::cout << "Payload at [" << position << "] has crashed into asteroid" << std::endl;
            active = false;
        }
        else if ((1.001 * effective_force_magnitude) >= position.squaredNorm())
        {
            std::cout << "Payload simulation boundary reached" << std::endl;
            active = false;
        }
    }
}

}
