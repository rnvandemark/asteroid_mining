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
        const double initial_bottom_lifting_side_mass_position,
        const double initial_bottom_lifting_side_mass_velocity
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
        max_bottom_lifting_side_mass_position(chain_length / n),
        siphon_angular_position(initial_siphon_angular_position),
        siphon_angular_velocity(initial_siphon_angular_velocity),
        siphon_angular_acceleration(0.0),
        bottom_lifting_side_mass_position(initial_bottom_lifting_side_mass_position),
        bottom_lifting_side_mass_velocity(initial_bottom_lifting_side_mass_velocity),
        bottom_lifting_side_mass_acceleration(0.0),
        cs_payload_mass(0.0),
        total_time_elapsed(0.0),
        time_mass_reached_cs_last(0.0),
        time_elapsed_last_mass_to_reach_cs(0.0),
        last_siphon_angular_velocity_was_positive(false),
        last_min_siphon_angular_position_reached(0.0),
        last_max_siphon_angular_position_reached(0.0),
        mass_positions(n),
        mass_effective_forces(n)
    {
        std::cout << "Siphon characteristics:" << std::endl;
        std::cout << "- number of payloads on each side: " << n << std::endl;
        std::cout << "- chain length: " << chain_length << std::endl;
        std::cout << "- max mass position: " << max_bottom_lifting_side_mass_position << std::endl;
        std::cout << "- bucket mass: " << bucket_mass << std::endl;
        std::cout << "- payload mass: " << payload_mass << std::endl;
        std::cout << "- CS dry mass: " << cs_dry_mass << std::endl;
        std::cout << "- anchor point angle: " << (anchor_point_polar_angle * 180 / M_PI) << std::endl;
        std::cout << "- anchor point radius: " << anchor_point_polar_radius << std::endl;
        std::cout << "- init siphon angular position: " << siphon_angular_position << std::endl;
        std::cout << "- init siphon angular velocity: " << siphon_angular_velocity << std::endl;
        std::cout << "- init mass position: " << bottom_lifting_side_mass_position << std::endl;
        std::cout << "- init mass velocity: " << bottom_lifting_side_mass_velocity << std::endl;
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
        const double initial_bottom_lifting_side_mass_position,
        const double initial_bottom_lifting_side_mass_velocity
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
            dimensions_scaler.get_dimensionless(initial_bottom_lifting_side_mass_position, Soc() * Sf(Dt::DISTANCE)),
            dimensions_scaler.get_dimensionless(initial_bottom_lifting_side_mass_velocity, Soc() / Sf(Dt::TIME))
        );
    }

    double get_siphon_angular_position() const
    {
        return siphon_angular_position;
    }

    double get_siphon_angular_velocity() const
    {
        return siphon_angular_velocity;
    }

    double get_siphon_angular_acceleration() const
    {
        return siphon_angular_acceleration;
    }

    double get_cs_payload_mass() const
    {
        return cs_payload_mass;
    }

    double get_time_elapsed_last_mass_to_reach_cs() const
    {
        return time_elapsed_last_mass_to_reach_cs;
    }

    double get_last_min_siphon_angular_position_reached() const
    {
        return last_min_siphon_angular_position_reached;
    }

    double get_last_max_siphon_angular_position_reached() const
    {
        return last_max_siphon_angular_position_reached;
    }

    bool get_mass_is_lifting(const unsigned int i) const
    {
        return i < n;
    }

    double get_mass_position(const unsigned int i) const
    {
        return mass_positions[i];
    }

    const std::array<double, 3>& get_mass_effective_force(const unsigned int i) const
    {
        return mass_effective_forces[i];
    }

    void clear_cs_payload_mass()
    {
        cs_payload_mass = 0.0;
    }

    virtual void progress_over(const double dt) override
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

        const double net_chain_angle = anchor_point_polar_angle + siphon_angular_position;
        const double cs_total_mass = cs_payload_mass + cs_dry_mass;
        const auto cs_effective_force = calculate_cartesian_effective_force_on_chain_at(chain_length);

        double net_radial_force = 0;
        double net_torque = cs_total_mass * chain_length * (
            (cs_effective_force[0] * -std::sin(net_chain_angle)) + (cs_effective_force[1] * std::cos(net_chain_angle))
        );
        double net_moment = cs_total_mass * chain_length * chain_length;
        for (std::size_t m = 0; m < n; m++)
        {
            const double mass_radial_force = lifting_side_mass * (
                ((mass_effective_forces[m][0] * std::cos(net_chain_angle)) + (mass_effective_forces[m][1] * std::sin(net_chain_angle)))
                + (mass_positions[m] * siphon_angular_velocity * siphon_angular_velocity)
                + (2 * siphon_angular_velocity * mass_positions[m])
            );
            const double mass_torque = mass_positions[m] * (
                ((mass_effective_forces[m][0] * -std::sin(net_chain_angle)) + (mass_effective_forces[m][1] * std::cos(net_chain_angle)))
                - (2 * (1 + siphon_angular_velocity) * bottom_lifting_side_mass_velocity)
            );
            const double mass_moment = lifting_side_mass * mass_positions[m] * mass_positions[m];

            net_radial_force += mass_radial_force;
            net_torque += mass_torque;
            net_moment += mass_moment;
        }
        for (std::size_t m = n; m < 2 * n; m++)
        {
            const double mass_radial_force = -descending_side_mass * (
                ((mass_effective_forces[m][0] * std::cos(net_chain_angle)) + (mass_effective_forces[m][1] * std::sin(net_chain_angle)))
                + (mass_positions[m] * siphon_angular_velocity * siphon_angular_velocity)
                + (2 * siphon_angular_velocity * mass_positions[m])
            );
            const double mass_torque = mass_positions[m] * descending_side_mass * (
                ((mass_effective_forces[m][1] * -std::sin(net_chain_angle)) + (mass_effective_forces[m][1] * std::cos(net_chain_angle)))
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

    easy3d::vec3 get_position_in_asteroid_frame(const double chain_position) const
    {
        const double net_chain_angle = anchor_point_polar_angle + siphon_angular_position;
        return easy3d::vec3(
            (anchor_point_polar_radius * std::cos(anchor_point_polar_angle)) + (chain_position * std::cos(net_chain_angle)),
            (anchor_point_polar_radius * std::sin(anchor_point_polar_angle)) + (chain_position * std::sin(net_chain_angle)),
            0
        );
    }

    std::array<double, 3> calculate_cartesian_effective_force_on_chain_at(const double chain_position) const
    {
        double dummy;
        return calculate_cartesian_effective_force_on_chain_at(chain_position, dummy);
    }

    std::array<double, 3> calculate_cartesian_effective_force_on_chain_at(const double chain_position, double& magnitude) const
    {
        return asteroid.calculate_cartesian_effective_force_at(get_position_in_asteroid_frame(chain_position), magnitude);
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
    std::vector<std::array<double, 3>> mass_effective_forces;
};

}
