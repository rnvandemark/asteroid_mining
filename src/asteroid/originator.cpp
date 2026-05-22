#include "asteroid_mining/asteroid/originator.hpp"

#include "asteroid_mining/constants.hpp"
#include "asteroid_mining/math.hpp"

namespace asteroid_mining {

AsteroidOriginator::AsteroidOriginator(
    const double beta_,
    const double gamma_,
    const double density_,
    const double rho_A,
    const double omega_bar
):
    AsteroidCarrier(
        1.0,
        beta_,
        gamma_,
        density_,
        1.0,
        omega_bar / std::sqrt(4 * M_PI * constants::G * rho_A * beta_ * gamma_ / 3),
        0.0
    )
{
}

AsteroidOriginator AsteroidOriginator::from_dimensioned_values(
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
    return AsteroidOriginator(
        dimensions_scaler.get_dimensionless(beta_bar, Soc() * Sf(Dt::DISTANCE)),
        dimensions_scaler.get_dimensionless(gamma_bar, Soc() * Sf(Dt::DISTANCE)),
        dimensions_scaler.get_dimensionless(rho_A, Soc() * Sf(Dt::MASS) / Sf(Dt::DISTANCE, 3)),
        rho_A,
        omega_bar
    );
}

const AsteroidCarrier& AsteroidOriginator::get_state() const
{
    return static_cast<const AsteroidCarrier&>(*this);
}

void AsteroidOriginator::progress_over(const double dt)
{
    rotation += (dt * angular_velocity);
}

bool AsteroidOriginator::is_point_within(const Eigen::Vector3d& p) const
{
    const double x = p.x() / alpha;
    const double y = p.y() / beta;
    const double z = p.z() / gamma;
    return (x*x) + (y*y) + (z*z) + 1e-6 <= 1.0;
}

Eigen::Vector3d AsteroidOriginator::calculate_cartesian_effective_force_at(const Eigen::Vector3d& position) const
{
    double dummy;
    return calculate_cartesian_effective_force_at(position, dummy);
}

Eigen::Vector3d AsteroidOriginator::calculate_cartesian_effective_force_at(const Eigen::Vector3d& position, double& magnitude) const
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
