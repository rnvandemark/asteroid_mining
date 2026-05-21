#include "asteroid_mining/math.hpp"

#include <boost/math/special_functions/ellint_rd.hpp>

namespace asteroid_mining { namespace math {

double round(const double v, const unsigned int n)
{
    const double f = std::pow(10, n);
    return std::round(v * f) / f;
}

Eigen::Matrix3d rotation_to_align(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1)
{
    const Eigen::Vector3d v0n = v0.normalized();
    const Eigen::Vector3d v1n = v1.normalized();

    const Eigen::Vector3d v = v0n.cross(v1n);
    const Eigen::Matrix3d vx{
        { 0,    -v[2], +v[1]},
        {+v[2],  0,    -v[0]},
        {-v[1], +v[0],  0   },
    };
    return Eigen::Matrix3d::Identity() + vx + ((vx * vx) / (1 + v0n.dot(v1n)));
}

double calculate_confocal_ellipsoid_surface(
    const double beta,
    const double gamma,
    const double x,
    const double y,
    const double z
)
{
    const double beta2 = beta * beta;
    const double gamma2 = gamma * gamma;
    const double x2 = x * x;
    const double y2 = y * y;
    const double z2 = z * z;

    const auto roots = cubic_roots(
        1.0,
        1.0 + beta2 + gamma2 - x2 - y2 - z2,
        beta2 + gamma2 + (beta2 * gamma2) - (x2 * beta2) - (x2 * gamma2) - y2 - (y2 * gamma2) - z2 - (z2 * beta2),
        (beta2 * gamma2) - (x2 * beta2 * gamma2) - (gamma2 * y2) - (z2 * beta2)
    );

    double lambda = roots[0];
    for (unsigned int i = 1; i < roots.size(); i++)
    {
        if ((!std::isnan(roots[i])) && (roots[i] > lambda))
        {
            lambda = roots[i];
        }
    }

    return lambda;
}

Eigen::Vector3d calculate_cartesian_effective_force(
    const double beta,
    const double gamma,
    const double omega,
    const double x,
    const double y,
    const double z,
    const double lambda
)
{
    const double c = 1 / (omega * omega);
    const double rd0 = 1 + lambda;
    const double rd1 = (beta * beta) + lambda;
    const double rd2 = (gamma * gamma) + lambda;
    return Eigen::Vector3d{
        x * (1 - (c * boost::math::ellint_rd(rd1, rd2, rd0))),
        y * (1 - (c * boost::math::ellint_rd(rd2, rd0, rd1))),
        z * (1 - (c * boost::math::ellint_rd(rd0, rd1, rd2)))
    };
}

Eigen::Vector3d calculate_cartesian_effective_force(
    const double beta,
    const double gamma,
    const double omega,
    const double x,
    const double y,
    const double z,
    const double lambda,
    double& magnitude
)
{
    const auto effective_forces = calculate_cartesian_effective_force(beta, gamma, omega, x, y, z, lambda);
    magnitude = effective_forces.norm();
    return effective_forces;
}

}}
