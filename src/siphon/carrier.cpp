#include "asteroid_mining/siphon/carrier.hpp"

namespace asteroid_mining {

SiphonCarrier::SiphonCarrier():
    SiphonCarrier(
        0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        false,
        0.0,
        0.0,
        {},
        {}
    )
{
}

SiphonCarrier::SiphonCarrier(
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
):
    n(n_),
    chain_length(chain_length_),
    bucket_mass(bucket_mass_),
    payload_mass(payload_mass_),
    lifting_side_mass(lifting_side_mass_),
    descending_side_mass(descending_side_mass_),
    cs_dry_mass(cs_dry_mass_),
    anchor_point_polar_angle(anchor_point_polar_angle_),
    anchor_point_polar_radius(anchor_point_polar_radius_),
    max_bottom_lifting_side_mass_position(max_bottom_lifting_side_mass_position_),
    siphon_angular_position(siphon_angular_position_),
    siphon_angular_velocity(siphon_angular_velocity_),
    siphon_angular_acceleration(siphon_angular_acceleration_),
    bottom_lifting_side_mass_position(bottom_lifting_side_mass_position_),
    bottom_lifting_side_mass_velocity(bottom_lifting_side_mass_velocity_),
    bottom_lifting_side_mass_acceleration(bottom_lifting_side_mass_acceleration_),
    cs_payload_mass(cs_payload_mass_),
    total_time_elapsed(total_time_elapsed_),
    time_mass_reached_cs_last(time_mass_reached_cs_last_),
    time_elapsed_last_mass_to_reach_cs(time_elapsed_last_mass_to_reach_cs_),
    last_siphon_angular_velocity_was_positive(last_siphon_angular_velocity_was_positive_),
    last_min_siphon_angular_position_reached(last_min_siphon_angular_position_reached_),
    last_max_siphon_angular_position_reached(last_max_siphon_angular_position_reached_),
    mass_positions(mass_positions_),
    mass_effective_forces(mass_effective_forces_)
{
}

unsigned int SiphonCarrier::get_n() const
{
    return n;
}

double SiphonCarrier::get_chain_length() const
{
    return chain_length;
}
double SiphonCarrier::get_bucket_mass() const
{
    return bucket_mass;
}
double SiphonCarrier::get_payload_mass() const
{
    return payload_mass;
}
double SiphonCarrier::get_lifting_side_mass() const
{
    return lifting_side_mass;
}
double SiphonCarrier::get_descending_side_mass() const
{
    return descending_side_mass;
}

double SiphonCarrier::get_cs_dry_mass() const
{
    return cs_dry_mass;
}

double SiphonCarrier::get_anchor_point_polar_angle() const
{
    return anchor_point_polar_angle;
}
double SiphonCarrier::get_anchor_point_polar_radius() const
{
    return anchor_point_polar_radius;
}

double SiphonCarrier::get_max_bottom_lifting_side_mass_position() const
{
    return max_bottom_lifting_side_mass_position;
}

double SiphonCarrier::get_siphon_angular_position() const
{
    return siphon_angular_position;
}
double SiphonCarrier::get_siphon_angular_velocity() const
{
    return siphon_angular_velocity;
}
double SiphonCarrier::get_siphon_angular_acceleration() const
{
    return siphon_angular_acceleration;
}

double SiphonCarrier::get_bottom_lifting_side_mass_position() const
{
    return bottom_lifting_side_mass_position;
}
double SiphonCarrier::get_bottom_lifting_side_mass_velocity() const
{
    return bottom_lifting_side_mass_velocity;
}
double SiphonCarrier::get_bottom_lifting_side_mass_acceleration() const
{
    return bottom_lifting_side_mass_acceleration;
}

double SiphonCarrier::get_cs_payload_mass() const
{
    return cs_payload_mass;
}

double SiphonCarrier::get_total_time_elapsed() const
{
    return total_time_elapsed;
}
double SiphonCarrier::get_time_mass_reached_cs_last() const
{
    return time_mass_reached_cs_last;
}
double SiphonCarrier::get_time_elapsed_last_mass_to_reach_cs() const
{
    return time_elapsed_last_mass_to_reach_cs;
}

bool SiphonCarrier::get_last_siphon_angular_velocity_was_positive() const
{
    return last_siphon_angular_velocity_was_positive;
}
double SiphonCarrier::get_last_min_siphon_angular_position_reached() const
{
    return last_min_siphon_angular_position_reached;
}
double SiphonCarrier::get_last_max_siphon_angular_position_reached() const
{
    return last_max_siphon_angular_position_reached;
}

double SiphonCarrier::get_net_chain_angle() const
{
    return anchor_point_polar_angle + siphon_angular_position;
}

bool SiphonCarrier::get_mass_is_lifting(const unsigned int i) const
{
    return i < n;
}

double SiphonCarrier::get_mass_position(const unsigned int i) const
{
    return mass_positions[i];
}

const Eigen::Vector3d& SiphonCarrier::get_mass_effective_force(const unsigned int i) const
{
    return mass_effective_forces[i];
}

}
