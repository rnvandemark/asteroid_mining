#include "asteroid.hpp"
#include "dimensions_scaler.hpp"
#include "model.hpp"
#include "siphon.hpp"
#include "view.hpp"

#include <easy3d/util/initializer.h>

int main(int argc, char** argv)
{
    const double alpha_bar = 280;  // m
    const double beta_bar = 140;  // m
    const double gamma_bar = 140;  // m
    const double rho_bar = 2500.0;  // kg/m^3
    const double omega_bar = 3.490659e-4;  // rad/s

    const unsigned int num_payloads_per_side = 35;
    const double chain_length = 270;
    const double bucket_mass = 0.0;
    const double payload_mass = 20.0;
    const double cs_dry_mass = 2000.0;
    const double anchor_point_polar_angle = 0;
    const double initial_siphon_angular_position = 0.0;
    const double initial_siphon_angular_velocity = 0.0;
    const double initial_bottom_lifting_side_mass_position = 0.0;
    const double initial_bottom_lifting_side_mass_velocity = 0.0;

    const double siphon_width = 5;
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
        num_payloads_per_side,
        chain_length,
        bucket_mass,
        payload_mass,
        cs_dry_mass,
        anchor_point_polar_angle,
        initial_siphon_angular_position,
        initial_siphon_angular_velocity,
        initial_bottom_lifting_side_mass_position,
        initial_bottom_lifting_side_mass_velocity
    );
    am::Model model(asteroid, siphon);

    easy3d::initialize();
    return am::View(
        dimensions_scaler,
        model,
        siphon_width,
        gravity_gradient_shell_radius,
        gravity_gradient_phi_step,
        gravity_gradient_theta_step
    ).run();
}
