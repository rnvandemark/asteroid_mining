#include "asteroid_mining/siphon/originator.hpp"

#include "asteroid_mining/math.hpp"

#include <cmath>

namespace asteroid_mining {

SiphonOriginator::SiphonOriginator(
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
):
    SiphonCarrier(
        n_,
        chain_length_,
        bucket_mass_,
        payload_mass_,
        bucket_mass_ + payload_mass_,
        bucket_mass_,
        cs_dry_mass_,
        anchor_point_polar_angle_,
        asteroid_.get_beta() / std::sqrt(1 - ((1 - std::pow(asteroid_.get_beta(), 2)) * std::pow(std::cos(anchor_point_polar_angle_), 2))),
        chain_length_ / n_,
        initial_siphon_angular_position,
        initial_siphon_angular_velocity,
        0.0,
        initial_bottom_lifting_side_mass_position,
        initial_bottom_lifting_side_mass_velocity,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        false,
        0.0,
        0.0,
        std::vector<double>(n_),
        std::vector<Eigen::Vector3d>(n_)
    ),
    asteroid(asteroid_)
{
}

SiphonOriginator SiphonOriginator::from_dimensioned_values(
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
)
{
    using Soc = DimensionsScaler::ScaleOpChain;
    using Sf = DimensionsScaler::ScaleFactor;
    using Dt = Sf::DimensionType;
    return SiphonOriginator(
        asteroid,
        n,
        dimensions_scaler.get_dimensionless(chain_length, Soc() * Sf(Dt::DISTANCE)),
        dimensions_scaler.get_dimensionless(bucket_mass, Soc() * Sf(Dt::MASS)),
        dimensions_scaler.get_dimensionless(payload_mass, Soc() * Sf(Dt::MASS)),
        dimensions_scaler.get_dimensionless(cs_dry_mass, Soc() * Sf(Dt::MASS)),
        anchor_point_polar_angle,
        initial_siphon_angular_position,
        dimensions_scaler.get_dimensionless(initial_siphon_angular_velocity, Soc() / Sf(Dt::TIME)),
        dimensions_scaler.get_dimensionless(initial_bottom_lifting_side_mass_position, Soc() * Sf(Dt::DISTANCE)),
        dimensions_scaler.get_dimensionless(initial_bottom_lifting_side_mass_velocity, Soc() / Sf(Dt::TIME))
    );
}

const SiphonCarrier& SiphonOriginator::get_state() const
{
    return static_cast<const SiphonCarrier&>(*this);
}

void SiphonOriginator::clear_cs_payload_mass()
{
    cs_payload_mass = 0.0;
}

void SiphonOriginator::progress_over(const double dt)
{
    const double last_siphon_angular_position = siphon_angular_position;

    total_time_elapsed += dt;

    // Update the first mass' position and chain's angular position
    siphon_angular_position += (siphon_angular_velocity * dt) + (0.5 * siphon_angular_acceleration * dt * dt);
    bottom_lifting_side_mass_position += (bottom_lifting_side_mass_velocity * dt) + (0.5 * bottom_lifting_side_mass_acceleration * dt * dt);

    // Update the first mass' velocity and chain's angular velocity
    siphon_angular_velocity += siphon_angular_acceleration * dt;
    bottom_lifting_side_mass_velocity += bottom_lifting_side_mass_acceleration * dt;

    // Handle if a mass has reached the collecting satellite and a new mass
    // has become the "bottom lifting side mass"
    while (bottom_lifting_side_mass_position > max_bottom_lifting_side_mass_position)
    {
        cs_payload_mass += payload_mass;

        bottom_lifting_side_mass_position -= max_bottom_lifting_side_mass_position;
        bottom_lifting_side_mass_velocity *= ((n - 1) / n);

        time_elapsed_last_mass_to_reach_cs = total_time_elapsed - time_mass_reached_cs_last;
        time_mass_reached_cs_last = total_time_elapsed;
    }

    for (std::size_t m = 0; m < 2 * n; m++)
    {
        mass_positions[m] = (
            (m < n)
            ? (bottom_lifting_side_mass_position + (m * max_bottom_lifting_side_mass_position))
            : (chain_length - (bottom_lifting_side_mass_position + ((m - n) * max_bottom_lifting_side_mass_position)))
        );
        mass_effective_forces[m] = calculate_cartesian_effective_force_on_chain_at(mass_positions[m]);
    }

    const double net_chain_angle = get_net_chain_angle();
    const double cs_total_mass = cs_payload_mass + cs_dry_mass;
    const auto cs_effective_force = calculate_cartesian_effective_force_on_chain_at(chain_length);

    double net_radial_force = 0;
    double net_torque = cs_total_mass * chain_length * (
        (cs_effective_force.x() * -std::sin(net_chain_angle)) + (cs_effective_force.y() * std::cos(net_chain_angle))
    );
    double net_moment = cs_total_mass * chain_length * chain_length;
    for (std::size_t m = 0; m < n; m++)
    {
        const auto& mass_effective_force = mass_effective_forces[m];
        const double mass_radial_force = lifting_side_mass * (
            ((mass_effective_force.x() * std::cos(net_chain_angle)) + (mass_effective_force.y() * std::sin(net_chain_angle)))
            + (mass_positions[m] * siphon_angular_velocity * siphon_angular_velocity)
            + (2 * siphon_angular_velocity * mass_positions[m])
        );
        const double mass_torque = mass_positions[m] * (
            ((mass_effective_force.x() * -std::sin(net_chain_angle)) + (mass_effective_force.y() * std::cos(net_chain_angle)))
            - (2 * (1 + siphon_angular_velocity) * bottom_lifting_side_mass_velocity)
        );
        const double mass_moment = lifting_side_mass * mass_positions[m] * mass_positions[m];

        net_radial_force += mass_radial_force;
        net_torque += mass_torque;
        net_moment += mass_moment;
    }
    for (std::size_t m = n; m < 2 * n; m++)
    {
        const auto& mass_effective_force = mass_effective_forces[m];
        const double mass_radial_force = -descending_side_mass * (
            ((mass_effective_force.x() * std::cos(net_chain_angle)) + (mass_effective_force.y() * std::sin(net_chain_angle)))
            + (mass_positions[m] * siphon_angular_velocity * siphon_angular_velocity)
            + (2 * siphon_angular_velocity * mass_positions[m])
        );
        const double mass_torque = mass_positions[m] * descending_side_mass * (
            ((mass_effective_force.x() * -std::sin(net_chain_angle)) + (mass_effective_force.y() * std::cos(net_chain_angle)))
            + (2 * (1 + siphon_angular_velocity) * bottom_lifting_side_mass_velocity)
        );
        const double mass_moment = descending_side_mass * mass_positions[m] * mass_positions[m];

        net_radial_force += mass_radial_force;
        net_torque += mass_torque;
        net_moment += mass_moment;
    }

    bottom_lifting_side_mass_acceleration = net_radial_force / (n * (1 + descending_side_mass));
    siphon_angular_acceleration = net_torque / net_moment;

    if (last_siphon_angular_velocity_was_positive && (siphon_angular_velocity < 0))
    {
        last_max_siphon_angular_position_reached = last_siphon_angular_position;
        last_siphon_angular_velocity_was_positive = false;
    }
    else if ((!last_siphon_angular_velocity_was_positive) && (siphon_angular_velocity > 0))
    {
        last_min_siphon_angular_position_reached = last_siphon_angular_position;
        last_siphon_angular_velocity_was_positive = true;
    }
}

Eigen::Vector3d SiphonOriginator::get_position_in_asteroid_frame(const double chain_position) const
{
    const double net_chain_angle = get_net_chain_angle();
    return Eigen::Vector3d{
        (anchor_point_polar_radius * std::cos(anchor_point_polar_angle)) + (chain_position * std::cos(net_chain_angle)),
        (anchor_point_polar_radius * std::sin(anchor_point_polar_angle)) + (chain_position * std::sin(net_chain_angle)),
        0
    };
}

Eigen::Vector3d SiphonOriginator::calculate_cartesian_effective_force_on_chain_at(const double chain_position) const
{
    double dummy;
    return calculate_cartesian_effective_force_on_chain_at(chain_position, dummy);
}

Eigen::Vector3d SiphonOriginator::calculate_cartesian_effective_force_on_chain_at(const double chain_position, double& magnitude) const
{
    return asteroid.calculate_cartesian_effective_force_at(get_position_in_asteroid_frame(chain_position), magnitude);
}

}
