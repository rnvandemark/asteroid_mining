#include "asteroid_mining/siphon/memento.hpp"

namespace asteroid_mining {

void SiphonMemento::set_state(const SiphonCarrier& carrier)
{
    n = carrier.n;
    chain_length = carrier.chain_length;
    bucket_mass = carrier.bucket_mass;
    payload_mass = carrier.payload_mass;
    lifting_side_mass = carrier.lifting_side_mass;
    descending_side_mass = carrier.descending_side_mass;
    cs_dry_mass = carrier.cs_dry_mass;
    anchor_point_polar_angle = carrier.anchor_point_polar_angle;
    anchor_point_polar_radius = carrier.anchor_point_polar_radius;
    max_bottom_lifting_side_mass_position = carrier.max_bottom_lifting_side_mass_position;
    siphon_angular_position = carrier.siphon_angular_position;
    siphon_angular_velocity = carrier.siphon_angular_velocity;
    siphon_angular_acceleration = carrier.siphon_angular_acceleration;
    bottom_lifting_side_mass_position = carrier.bottom_lifting_side_mass_position;
    bottom_lifting_side_mass_velocity = carrier.bottom_lifting_side_mass_velocity;
    bottom_lifting_side_mass_acceleration = carrier.bottom_lifting_side_mass_acceleration;
    cs_payload_mass = carrier.cs_payload_mass;
    total_time_elapsed = carrier.total_time_elapsed;
    time_mass_reached_cs_last = carrier.time_mass_reached_cs_last;
    time_elapsed_last_mass_to_reach_cs = carrier.time_elapsed_last_mass_to_reach_cs;
    last_siphon_angular_velocity_was_positive = carrier.last_siphon_angular_velocity_was_positive;
    last_min_siphon_angular_position_reached = carrier.last_min_siphon_angular_position_reached;
    last_max_siphon_angular_position_reached = carrier.last_max_siphon_angular_position_reached;
    mass_positions = carrier.mass_positions;
    mass_effective_forces = carrier.mass_effective_forces;
}

}
