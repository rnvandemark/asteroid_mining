#include "asteroid_mining/math.hpp"

#include <boost/math/special_functions/ellint_rd.hpp>

namespace asteroid_mining { namespace math {

double round(const double v, const unsigned int n)
{
    const double f = std::pow(10, n);
    return std::round(v * f) / f;
}

easy3d::Mat3<float> rotation_to_align(const easy3d::vec3& v0, const easy3d::vec3& v1)
{
    const easy3d::vec3 v0n = easy3d::normalize(v0);
    const easy3d::vec3 v1n = easy3d::normalize(v1);

    const easy3d::vec3 v = easy3d::cross(v0n, v1n);
    const easy3d::Mat3<float> vx(
         0,    -v[2], +v[1],
        +v[2],  0,    -v[0],
        -v[1], +v[0],  0
    );
    return easy3d::Mat3<float>::identity() + vx + ((vx * vx) / (1 + easy3d::dot(v0n, v1n)));
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

std::array<double, 3> calculate_cartesian_effective_force(
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
    return std::array<double, 3>{
        x * (1 - (c * boost::math::ellint_rd(rd1, rd2, rd0))),
        y * (1 - (c * boost::math::ellint_rd(rd2, rd0, rd1))),
        z * (1 - (c * boost::math::ellint_rd(rd0, rd1, rd2)))
    };
}

std::array<double, 3> calculate_cartesian_effective_force(
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
    magnitude = std::sqrt(
        (effective_forces[0] * effective_forces[0])
        + (effective_forces[1] * effective_forces[1])
        + (effective_forces[2] * effective_forces[2])
    );
    return effective_forces;
}

}}
