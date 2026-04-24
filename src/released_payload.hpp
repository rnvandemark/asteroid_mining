#pragma once

#include "progressable_i.hpp"

namespace am {

class ReleasedPayload : public ProgressableI
{
public:
    ReleasedPayload(const Asteroid& asteroid_) :
        asteroid(asteroid_),
        active(false),
        mass(0.0),
        position(),
        velocity(),
        acceleration()
    {
    }

    bool is_active() const
    {
        return active;
    }

    const easy3d::vec3& get_position() const
    {
        return position;
    }

    void release(
        const double mass_,
        const easy3d::vec3& position_,
        const easy3d::vec3& velocity_,
        const easy3d::vec3& acceleration_
    )
    {
        mass = mass_;
        position = position_;
        velocity = velocity_;
        acceleration = acceleration_;
        active = true;
    }

    virtual void progress_over(const double dt) override
    {
        if (active)
        {
            position.x += (velocity.x * dt) + (0.5 * acceleration.x * dt * dt);
            position.y += (velocity.y * dt) + (0.5 * acceleration.y * dt * dt);
            position.z += (velocity.z * dt) + (0.5 * acceleration.z * dt * dt);

            velocity.x += (acceleration.x * dt);
            velocity.y += (acceleration.y * dt);
            velocity.z += (acceleration.z * dt);

            const auto effective_force = asteroid.calculate_cartesian_effective_force_at(position);

            acceleration.x = effective_force[0] + (2 * velocity.y);
            acceleration.y = effective_force[1] - (2 * velocity.x);
            acceleration.z = effective_force[2];

            if (asteroid.is_point_within(position))
            {
                std::cout << "Payload at [" << position << "] has crashed into asteroid" << std::endl;
                active = false;
            }
        }
    }

protected:
    const Asteroid& asteroid;

    bool active;

    double mass;
    easy3d::vec3 position;
    easy3d::vec3 velocity;
    easy3d::vec3 acceleration;
};

}
