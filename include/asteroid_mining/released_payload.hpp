#pragma once

#include "asteroid_mining/asteroid.hpp"

namespace asteroid_mining {

class ReleasedPayload : public ProgressableI
{
public:
    ReleasedPayload(const Asteroid& asteroid_);

    bool is_active() const;

    const easy3d::vec3& get_position() const;
    const easy3d::vec3& get_velocity() const;

    void release(
        const double mass_,
        const easy3d::vec3& position_,
        const easy3d::vec3& velocity_,
        const easy3d::vec3& acceleration_
    );

    virtual void progress_over(const double dt) override;

protected:
    const Asteroid& asteroid;

    bool active;

    double mass;
    easy3d::vec3 position;
    easy3d::vec3 velocity;
    easy3d::vec3 acceleration;
};

}
