#include "asteroid_mining/asteroid.hpp"

#include "asteroid_mining/math.hpp"

#include <iostream>

namespace asteroid_mining {

Asteroid::Asteroid(
    const double beta_,
    const double gamma_,
    const double density_,
    const double rho_A,
    const double omega_bar
):
    alpha(1.0),
    beta(beta_),
    gamma(gamma_),
    density(density_),
    angular_velocity(1.0),
    omega(omega_bar / std::sqrt(4 * M_PI * math::G * rho_A * beta * gamma / 3))
{
    std::cout << "Asteroid characteristics:" << std::endl;
    std::cout << "- alpha: " << alpha << std::endl;
    std::cout << "- beta: " << beta << std::endl;
    std::cout << "- gamma: " << gamma << std::endl;
    std::cout << "- density: " << density << std::endl;
    std::cout << "- angular velocity: " << angular_velocity << std::endl;
    std::cout << "- omega: " << omega << std::endl;
}

Asteroid Asteroid::from_dimensioned_values(
    const DimensionsScaler& dimensions_scaler,
    const double beta_bar,
    const double gamma_bar,
    const double rho_A,
    const double omega_bar
)
{
    using Soc = DimensionsScaler::ScaleOpChain;
    using Sf = DimensionsScaler::ScaleFactor;
    using Dt = Sf::DimensionType;
    return Asteroid(
        dimensions_scaler.get_dimensionless(beta_bar, Soc() * Sf(Dt::DISTANCE)),
        dimensions_scaler.get_dimensionless(gamma_bar, Soc() * Sf(Dt::DISTANCE)),
        dimensions_scaler.get_dimensionless(rho_A, Soc() * Sf(Dt::MASS) / Sf(Dt::DISTANCE, 3)),
        rho_A,
        omega_bar
    );
}

double Asteroid::get_rotation() const
{
    return rotation;
}

void Asteroid::progress_over(const double dt)
{
    rotation += (dt * angular_velocity);
}

bool Asteroid::is_point_within(const Eigen::Vector3d& p) const
{
    const double x = p.x() / alpha;
    const double y = p.y() / beta;
    const double z = p.z() / gamma;
    return (x*x) + (y*y) + (z*z) + 1e-6 <= 1.0;
}

Eigen::Vector3d Asteroid::calculate_cartesian_effective_force_at(const Eigen::Vector3d& position) const
{
    double dummy;
    return calculate_cartesian_effective_force_at(position, dummy);
}

Eigen::Vector3d Asteroid::calculate_cartesian_effective_force_at(const Eigen::Vector3d& position, double& magnitude) const
{
    const double lambda = ((position.squaredNorm() < 1e-6) ? 0 : math::calculate_confocal_ellipsoid_surface(
        beta,
        gamma,
        position.x(),
        position.y(),
        position.z()
    ));
    return math::calculate_cartesian_effective_force(
        beta,
        gamma,
        omega,
        position.x(),
        position.y(),
        position.z(),
        lambda,
        magnitude
    );
}

}
