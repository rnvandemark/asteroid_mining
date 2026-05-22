#pragma once

#include <Eigen/Dense>

namespace asteroid_mining {

class ReleasedPayloadCarrier
{
    friend class ReleasedPayloadMemento;

public:
    ReleasedPayloadCarrier();

    ReleasedPayloadCarrier(
        const bool active_,
        const double mass_,
        const Eigen::Vector3d& position_,
        const Eigen::Vector3d& velocity_,
        const Eigen::Vector3d& acceleration_
    );

    bool is_active() const;

    double get_mass() const;
    const Eigen::Vector3d& get_position() const;
    const Eigen::Vector3d& get_velocity() const;
    const Eigen::Vector3d& get_acceleration() const;

protected:
    bool active;

    double mass;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
};

}
