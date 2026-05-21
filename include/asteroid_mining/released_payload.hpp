#pragma once

#include "asteroid_mining/asteroid.hpp"

#include <Eigen/Dense>

namespace asteroid_mining {

class ReleasedPayload : public ProgressableI
{
public:
    ReleasedPayload(const Asteroid& asteroid_);

    bool is_active() const;

    const Eigen::Vector3d& get_position() const;
    const Eigen::Vector3d& get_velocity() const;

    void release(
        const double mass_,
        const Eigen::Vector3d& position_,
        const Eigen::Vector3d& velocity_,
        const Eigen::Vector3d& acceleration_
    );

    virtual void progress_over(const double dt) override;

protected:
    const Asteroid& asteroid;

    bool active;

    double mass;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
};

}
