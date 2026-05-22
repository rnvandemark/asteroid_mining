#pragma once

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
class AsteroidCarrier
{
    friend class AsteroidMemento;

public:
    AsteroidCarrier();

    AsteroidCarrier(
        const double alpha_,
        const double beta_,
        const double gamma_,
        const double density_,
        const double angular_velocity_,
        const double omega_,
        const double rotation_
    );

    double get_alpha() const;
    double get_beta() const;
    double get_gamma() const;

    double get_density() const;
    double get_angular_velocity() const;
    double get_omega() const;

    double get_rotation() const;

protected:
    // The longest dimension of the ellipsoid (distance).
    double alpha;
    // The intermediate dimension of the ellipsoid (distance).
    double beta;
    // The smallest dimension of the ellipsoid (distance).
    double gamma;

    // The asteroid's density (mass per cubic distance).
    double density;

    // The asteroid's angular velocity, (about the smallest dimension)
    // (rotation per time).
    double angular_velocity;

    double omega;

    // The current rotation/orientation, updated as time passes (rotation).
    double rotation;
};

}
