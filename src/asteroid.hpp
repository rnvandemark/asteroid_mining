#pragma once

#include "dimensions_scaler.hpp"
#include "math.hpp"
#include "progressable_i.hpp"

namespace am {

/* Triaxial ellipsoid with semi-major axes a_bar >= beta_bar >= gamma_bar,
 * constant density, constant angular velocity about the axis with largest
 * inertia. A co-rotating reference frame is defined such that the x-axis lies
 * along the largest dimension beta_bar and the z-axis lies along the smallest
 * dimension gamma_bar, parallel to the angular velocity vector.
 *
 * Non-dimensional variables are used. Distance variables are scaled by
 * alpha_bar (hence why this class does not take an alpha value as a parameter,
 * because alpha_bar scaled by itself is always 1).
 */
class Asteroid : public ProgressableI
{
public:
    Asteroid(
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
        omega(omega_bar / std::sqrt(4 * M_PI * G * rho_A * beta * gamma / 3))
    {
        std::cout << "Asteroid characteristics:" << std::endl;
        std::cout << "- alpha: " << alpha << std::endl;
        std::cout << "- beta: " << beta << std::endl;
        std::cout << "- gamma: " << gamma << std::endl;
        std::cout << "- density: " << density << std::endl;
        std::cout << "- angular velocity: " << angular_velocity << std::endl;
        std::cout << "- omega: " << omega << std::endl;
    }

    static Asteroid from_dimensioned_values(
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

    double get_rotation() const
    {
        return rotation;
    }

    virtual void progress_over(const double dt) override
    {
        rotation += (dt * angular_velocity);
    }

    std::array<double, 3> calculate_effective_potential_cartesian_partials_at(const easy3d::vec3& position) const
    {
        const double lambda = ((position.length() < 1e-6) ? 0 : calculate_confocal_ellipsoid_surface(
            beta,
            gamma,
            position.x,
            position.y,
            position.z
        ));
        return calculate_effective_potential_cartesian_partials(
            beta,
            gamma,
            omega,
            position.x,
            position.y,
            position.z,
            lambda
        );
    }

    // The longest dimension of the ellipsoid (distance).
    const double alpha;
    // The intermediate dimension of the ellipsoid (distance).
    const double beta;
    // The smallest dimension of the ellipsoid (distance).
    const double gamma;

    // The asteroid's density (mass per cubic distance).
    const double density;

    // The asteroid's angular velocity, (about the smallest dimension)
    // (rotation per time).
    const double angular_velocity;

    const double omega;

protected:
    // The current rotation/orientation, updated as time passes (rotation).
    double rotation;
};

}
