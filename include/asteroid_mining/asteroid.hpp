#pragma once

#include "asteroid_mining/dimensions_scaler.hpp"
#include "asteroid_mining/progressable_i.hpp"

#include <easy3d/core/types.h>

namespace asteroid_mining {

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
    );

    static Asteroid from_dimensioned_values(
        const DimensionsScaler& dimensions_scaler,
        const double beta_bar,
        const double gamma_bar,
        const double rho_A,
        const double omega_bar
    );

    double get_rotation() const;

    virtual void progress_over(const double dt) override;

    bool is_point_within(const easy3d::vec3& p) const;

    std::array<double, 3> calculate_cartesian_effective_force_at(const easy3d::vec3& position) const;
    std::array<double, 3> calculate_cartesian_effective_force_at(const easy3d::vec3& position, double& magnitude) const;

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
