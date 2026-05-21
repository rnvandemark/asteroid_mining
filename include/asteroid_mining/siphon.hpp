#pragma once

#include "asteroid_mining/asteroid.hpp"

#include <Eigen/Dense>

namespace asteroid_mining {

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
        const double initial_bottom_lifting_side_mass_position,
        const double initial_bottom_lifting_side_mass_velocity
    );

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
        const double initial_bottom_lifting_side_mass_position,
        const double initial_bottom_lifting_side_mass_velocity
    );

    double get_siphon_angular_position() const;
    double get_siphon_angular_velocity() const;
    double get_siphon_angular_acceleration() const;

    double get_cs_payload_mass() const;

    double get_time_elapsed_last_mass_to_reach_cs() const;

    double get_last_min_siphon_angular_position_reached() const;
    double get_last_max_siphon_angular_position_reached() const;

    bool get_mass_is_lifting(const unsigned int i) const;

    double get_mass_position(const unsigned int i) const;

    const Eigen::Vector3d& get_mass_effective_force(const unsigned int i) const;

    void clear_cs_payload_mass();

    virtual void progress_over(const double dt) override;

    Eigen::Vector3d get_position_in_asteroid_frame(const double chain_position) const;

    Eigen::Vector3d calculate_cartesian_effective_force_on_chain_at(const double chain_position) const;
    Eigen::Vector3d calculate_cartesian_effective_force_on_chain_at(const double chain_position, double& magnitude) const;

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

    const double max_bottom_lifting_side_mass_position;

protected:
    double siphon_angular_position;
    double siphon_angular_velocity;
    double siphon_angular_acceleration;

    double bottom_lifting_side_mass_position;
    double bottom_lifting_side_mass_velocity;
    double bottom_lifting_side_mass_acceleration;

    double cs_payload_mass;

    double total_time_elapsed;
    double time_mass_reached_cs_last;
    double time_elapsed_last_mass_to_reach_cs;

    bool last_siphon_angular_velocity_was_positive;
    double last_min_siphon_angular_position_reached;
    double last_max_siphon_angular_position_reached;

    std::vector<double> mass_positions;
    std::vector<Eigen::Vector3d> mass_effective_forces;
};

}
