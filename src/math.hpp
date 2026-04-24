#pragma once

#include <boost/math/special_functions/ellint_rd.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/math/tools/roots.hpp>
#include <easy3d/core/types.h>
#include <easy3d/core/mat.h>

namespace am {

    const double G = 6.67e-11;

    // DISCLAIMER: this is copy and paste from Boost
    // (https://github.com/boostorg/math/blob/develop/include/boost/math/tools/cubic_roots.hpp)
    // because it was introduced in a newer version that I don't have access to :(
    template <typename Real>
    std::array<Real, 3> cubic_roots(Real a, Real b, Real c, Real d) {
        std::array<Real, 3> roots = {std::numeric_limits<Real>::quiet_NaN(),
                                    std::numeric_limits<Real>::quiet_NaN(),
                                    std::numeric_limits<Real>::quiet_NaN()};
        if (a == 0) {
            // bx^2 + cx + d = 0:
            if (b == 0) {
                // cx + d = 0:
                if (c == 0) {
                    if (d != 0) {
                        // No solutions:
                        return roots;
                    }
                    roots[0] = 0;
                    roots[1] = 0;
                    roots[2] = 0;
                    return roots;
                }
                roots[0] = -d / c;
                return roots;
            }
            auto [x0, x1] = boost::math::tools::quadratic_roots(b, c, d);
            roots[0] = x0;
            roots[1] = x1;
            return roots;
        }
        if (d == 0) {
            auto [x0, x1] = boost::math::tools::quadratic_roots(a, b, c);
            roots[0] = x0;
            roots[1] = x1;
            roots[2] = 0;
            std::sort(roots.begin(), roots.end());
            return roots;
        }
        Real p = b / a;
        Real q = c / a;
        Real r = d / a;
        Real Q = (p * p - 3 * q) / 9;
        Real R = (2 * p * p * p - 9 * p * q + 27 * r) / 54;
        if (R * R < Q * Q * Q) {
            Real rtQ = std::sqrt(Q);
            Real v = R / (Q * rtQ);
            Real theta = 0;
            if (v > 1)
            {
                theta = 0;
            }
            else if (v < 1)
            {
                theta = M_PI;
            }
            else
            {
                Real theta = std::acos(v) / 3;
            }
            Real st = std::sin(theta);
            Real ct = std::cos(theta);
            roots[0] = -2 * rtQ * ct - p / 3;
            roots[1] = -rtQ * (-ct + std::sqrt(Real(3)) * st) - p / 3;
            roots[2] = rtQ * (ct + std::sqrt(Real(3)) * st) - p / 3;
        } else {
            // In Numerical Recipes, Chapter 5, Section 6, it is claimed that we
            // only have one real root if R^2 >= Q^3. But this isn't true; we can
            // even see this from equation 5.6.18. The condition for having three
            // real roots is that A = B. It *is* the case that if we're in this
            // branch, and we have 3 real roots, two are a double root. Take
            // (x+1)^2(x-2) = x^3 - 3x -2 as an example. This clearly has a double
            // root at x = -1, and it gets sent into this branch.
            Real arg = R * R - Q * Q * Q;
            Real A = (R >= 0 ? -1 : 1) * std::cbrt(std::abs(R) + std::sqrt(arg));
            Real B = 0;
            if (A != 0) {
                B = Q / A;
            }
            roots[0] = A + B - p / 3;
            // Yes, we're comparing floats for equality:
            // Any perturbation pushes the roots into the complex plane; out of the
            // bailiwick of this routine.
            if (A == B || arg == 0) {
                roots[1] = -A - p / 3;
                roots[2] = -A - p / 3;
            }
        }
        // Root polishing:
        for (auto &r : roots) {
            // Horner's method.
            // Here I'll take John Gustaffson's opinion that the fma is a *distinct*
            // operation from a*x +b: Make sure to compile these fmas into a single
            // instruction and not a function call! (I'm looking at you Windows.)
            Real f = std::fma(a, r, b);
            f = std::fma(f, r, c);
            f = std::fma(f, r, d);
            Real df = std::fma(3 * a, r, 2 * b);
            df = std::fma(df, r, c);
            if (df != 0) {
                Real d2f = std::fma(6 * a, r, 2 * b);
                Real denom = 2 * df * df - f * d2f;
                if (denom != 0) {
                    r -= 2 * f * df / denom;
                } else {
                    r -= f / df;
                }
            }
        }
        std::sort(roots.begin(), roots.end());
        return roots;
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
        const double c = 3 / (2 * omega * omega);
        const double rd0 = 1 + lambda;
        const double rd1 = (beta * beta) + lambda;
        const double rd2 = (gamma * gamma) + lambda;
        return std::array<double, 3>{
            x * (1 - (c * boost::math::ellint_rd(rd1, rd2, rd0))),
            y * (1 - (c * boost::math::ellint_rd(rd0, rd2, rd1))),
            z * (1 - (c * boost::math::ellint_rd(rd0, rd1, rd2)))
        };
    }

}
