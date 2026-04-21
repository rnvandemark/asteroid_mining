#pragma once

#include "asteroid.hpp"
#include "progressable_i.hpp"

#include <cmath>

namespace am {

class Siphon : public ProgressableI
{
public:
    Siphon(
        const Asteroid& asteroid_,
        const unsigned int n_,
        const double chain_length_,
        const double bucket_mass_,
        const double payload_mass_,
        const double cs_dry_mass_,
        const double anchor_point_polar_angle_,
        const double initial_siphon_angular_position,
        const double initial_siphon_angular_velocity,
        const double initial_siphon_angular_acceleration
    ):
        asteroid(asteroid_),
        n(n_),
        chain_length(chain_length_),
        bucket_mass(bucket_mass_),
        payload_mass(payload_mass_),
        lifting_side_mass(bucket_mass + payload_mass),
        descending_side_mass(bucket_mass),
        cs_dry_mass(cs_dry_mass_),
        anchor_point_polar_angle(anchor_point_polar_angle_),
        anchor_point_polar_radius(asteroid.beta / std::sqrt(1 - ((1 - std::pow(asteroid.beta, 2)) * std::pow(std::cos(anchor_point_polar_angle), 2)))),
        siphon_angular_position(initial_siphon_angular_position),
        siphon_angular_velocity(initial_siphon_angular_velocity),
        siphon_angular_acceleration(initial_siphon_angular_acceleration),
        cs_payload_mass(0.0)
    {
    }

    static Siphon from_dimensioned_values(
        const DimensionsScaler& dimensions_scaler,
        const Asteroid& asteroid,
        const unsigned int n,
        const double chain_length,
        const double bucket_mass,
        const double payload_mass,
        const double cs_dry_mass,
        const double anchor_point_polar_angle,
        const double initial_siphon_angular_position,
        const double initial_siphon_angular_velocity,
        const double initial_siphon_angular_acceleration
    )
    {
        using Soc = DimensionsScaler::ScaleOpChain;
        using Sf = DimensionsScaler::ScaleFactor;
        using Dt = Sf::DimensionType;
        return Siphon(
            asteroid,
            n,
            dimensions_scaler.get_dimensionless(chain_length, Soc() * Sf(Dt::DISTANCE)),
            dimensions_scaler.get_dimensionless(bucket_mass, Soc() * Sf(Dt::MASS)),
            dimensions_scaler.get_dimensionless(payload_mass, Soc() * Sf(Dt::MASS)),
            dimensions_scaler.get_dimensionless(cs_dry_mass, Soc() * Sf(Dt::MASS)),
            anchor_point_polar_angle,
            initial_siphon_angular_position,
            dimensions_scaler.get_dimensionless(initial_siphon_angular_velocity, Soc() / Sf(Dt::TIME)),
            dimensions_scaler.get_dimensionless(initial_siphon_angular_acceleration, Soc() / Sf(Dt::TIME, 2))
        );
    }

    virtual void progress_over(const double dt) override
    {

    }

    const Asteroid& asteroid;

    // The number of buckets on each side of the chain, so there are 2n total.
    const unsigned int n;

    const double chain_length;
    const double bucket_mass;
    const double payload_mass;
    const double lifting_side_mass;
    const double descending_side_mass;

    const double cs_dry_mass;

    const double anchor_point_polar_angle;
    const double anchor_point_polar_radius;

protected:
    double siphon_angular_position;
    double siphon_angular_velocity;
    double siphon_angular_acceleration;

    double cs_payload_mass;
};

}
