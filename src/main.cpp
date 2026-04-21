#include "asteroid.hpp"
#include "dimensions_scaler.hpp"
#include "model.hpp"
#include "siphon.hpp"
#include "view.hpp"

#include <easy3d/util/initializer.h>

int main(int argc, char** argv)
{
    const double alpha_bar = 282.5;  // m
    const double beta_bar = 267.5;  // m
    const double gamma_bar = 254.0;  // m
    const double rho_bar = 1260.0;  // kg/m^3
    const double omega_bar = 0.0004062678046;  // rad/s

    const unsigned int num_payloads = 4;
    const double chain_length = 500;
    const double bucket_mass = 50.0;
    const double payload_mass = 200.0;
    const double cs_dry_mass = 1000.0;
    const double anchor_point_polar_angle = M_PI/6;
    const double initial_siphon_angular_position = 0.0;
    const double initial_siphon_angular_velocity = 0.0;
    const double initial_siphon_angular_acceleration = 0.0;

    const double gravity_gradient_shell_radius = 2;
    const unsigned int gravity_gradient_phi_step = 20;
    const unsigned int gravity_gradient_theta_step = 20;

    am::DimensionsScaler dimensions_scaler(alpha_bar, bucket_mass + payload_mass, 1 / omega_bar);

    am::Asteroid asteroid = am::Asteroid::from_dimensioned_values(
        dimensions_scaler,
        beta_bar,
        gamma_bar,
        rho_bar,
        omega_bar
    );
    am::Siphon siphon = am::Siphon::from_dimensioned_values(
        dimensions_scaler,
        asteroid,
        num_payloads,
        chain_length,
        bucket_mass,
        payload_mass,
        cs_dry_mass,
        anchor_point_polar_angle,
        initial_siphon_angular_position,
        initial_siphon_angular_velocity,
        initial_siphon_angular_acceleration
    );
    am::Model model(asteroid, siphon);

    easy3d::initialize();
    return am::View(
        dimensions_scaler,
        model,
        gravity_gradient_shell_radius,
        gravity_gradient_phi_step,
        gravity_gradient_theta_step
    ).run();
}
