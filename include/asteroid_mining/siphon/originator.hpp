#pragma once

#include "asteroid_mining/asteroid/originator.hpp"
#include "asteroid_mining/originator_i.hpp"
#include "asteroid_mining/progressable_i.hpp"
#include "asteroid_mining/siphon/carrier.hpp"

namespace asteroid_mining {

class SiphonOriginator : public SiphonCarrier, public OriginatorI<SiphonCarrier>, public ProgressableI
{
public:
    SiphonOriginator(
        const AsteroidOriginator& asteroid_,
        const unsigned int n_,
        const double chain_length_,
        const double bucket_mass_,
        const double payload_mass_,
        const double cs_dry_mass_,
        const double anchor_point_polar_angle_,
        const double initial_siphon_angular_position,
        const double initial_siphon_angular_velocity,
        const double initial_bottom_lifting_side_mass_position,
        const double initial_bottom_lifting_side_mass_velocity
    );

    static SiphonOriginator from_dimensioned_values(
        const DimensionsScaler& dimensions_scaler,
        const AsteroidOriginator& asteroid,
        const unsigned int n,
        const double chain_length,
        const double bucket_mass,
        const double payload_mass,
        const double cs_dry_mass,
        const double anchor_point_polar_angle,
        const double initial_siphon_angular_position,
        const double initial_siphon_angular_velocity,
        const double initial_bottom_lifting_side_mass_position,
        const double initial_bottom_lifting_side_mass_velocity
    );

    virtual const SiphonCarrier& get_state() const override;

    virtual void progress_over(const double dt) override;

    void clear_cs_payload_mass();

    Eigen::Vector3d get_position_in_asteroid_frame(const double chain_position) const;

    Eigen::Vector3d calculate_cartesian_effective_force_on_chain_at(const double chain_position) const;
    Eigen::Vector3d calculate_cartesian_effective_force_on_chain_at(const double chain_position, double& magnitude) const;

    const AsteroidOriginator& asteroid;
};

}
