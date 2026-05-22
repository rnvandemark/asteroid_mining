#pragma once

#include <Eigen/Dense>

namespace asteroid_mining {

class SiphonCarrier
{
    friend class SiphonMemento;

public:
    SiphonCarrier();

    SiphonCarrier(
        const unsigned int n_,
        const double chain_length_,
        const double bucket_mass_,
        const double payload_mass_,
        const double lifting_side_mass_,
        const double descending_side_mass_,
        const double cs_dry_mass_,
        const double anchor_point_polar_angle_,
        const double anchor_point_polar_radius_,
        const double max_bottom_lifting_side_mass_position_,
        const double siphon_angular_position_,
        const double siphon_angular_velocity_,
        const double siphon_angular_acceleration_,
        const double bottom_lifting_side_mass_position_,
        const double bottom_lifting_side_mass_velocity_,
        const double bottom_lifting_side_mass_acceleration_,
        const double cs_payload_mass_,
        const double total_time_elapsed_,
        const double time_mass_reached_cs_last_,
        const double time_elapsed_last_mass_to_reach_cs_,
        const bool last_siphon_angular_velocity_was_positive_,
        const double last_min_siphon_angular_position_reached_,
        const double last_max_siphon_angular_position_reached_,
        const std::vector<double>& mass_positions_,
        const std::vector<Eigen::Vector3d>& mass_effective_forces_
    );

    unsigned int get_n() const;

    double get_chain_length() const;
    double get_bucket_mass() const;
    double get_payload_mass() const;
    double get_lifting_side_mass() const;
    double get_descending_side_mass() const;

    double get_cs_dry_mass() const;

    double get_anchor_point_polar_angle() const;
    double get_anchor_point_polar_radius() const;

    double get_max_bottom_lifting_side_mass_position() const;

    double get_siphon_angular_position() const;
    double get_siphon_angular_velocity() const;
    double get_siphon_angular_acceleration() const;

    double get_bottom_lifting_side_mass_position() const;
    double get_bottom_lifting_side_mass_velocity() const;
    double get_bottom_lifting_side_mass_acceleration() const;

    double get_cs_payload_mass() const;

    double get_total_time_elapsed() const;
    double get_time_mass_reached_cs_last() const;
    double get_time_elapsed_last_mass_to_reach_cs() const;

    bool get_last_siphon_angular_velocity_was_positive() const;
    double get_last_min_siphon_angular_position_reached() const;
    double get_last_max_siphon_angular_position_reached() const;

    double get_net_chain_angle() const;

    bool get_mass_is_lifting(const unsigned int i) const;

    double get_mass_position(const unsigned int i) const;

    const Eigen::Vector3d& get_mass_effective_force(const unsigned int i) const;

protected:
    // The number of buckets on each side of the chain, so there are 2n total.
    unsigned int n;

    double chain_length;
    double bucket_mass;
    double payload_mass;
    double lifting_side_mass;
    double descending_side_mass;

    double cs_dry_mass;

    double anchor_point_polar_angle;
    double anchor_point_polar_radius;

    double max_bottom_lifting_side_mass_position;

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
