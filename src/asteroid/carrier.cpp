#include "asteroid_mining/asteroid/carrier.hpp"

namespace asteroid_mining {

AsteroidCarrier::AsteroidCarrier():
    AsteroidCarrier(
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0
    )
{
}
AsteroidCarrier::AsteroidCarrier(
    const double alpha_,
    const double beta_,
    const double gamma_,
    const double density_,
    const double angular_velocity_,
    const double omega_,
    const double rotation_
):
    alpha(alpha_),
    beta(beta_),
    gamma(gamma_),
    density(density_),
    angular_velocity(angular_velocity_),
    omega(omega_),
    rotation(rotation_)
{
}

double AsteroidCarrier::get_alpha() const
{
    return alpha;
}
double AsteroidCarrier::get_beta() const
{
    return beta;
}
double AsteroidCarrier::get_gamma() const
{
    return gamma;
}

double AsteroidCarrier::get_density() const
{
    return density;
}
double AsteroidCarrier::get_angular_velocity() const
{
    return angular_velocity;
}
double AsteroidCarrier::get_omega() const
{
    return omega;
}

double AsteroidCarrier::get_rotation() const
{
    return rotation;
}

}
