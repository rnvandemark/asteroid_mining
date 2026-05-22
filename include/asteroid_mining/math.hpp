#pragma once

#include <boost/math/special_functions/sign.hpp>
#include <boost/math/tools/roots.hpp>
#include <Eigen/Dense>

#include <array>

namespace asteroid_mining { namespace math {

// DISCLAIMER: this is copy and paste from Boost
// (https://github.com/boostorg/math/blob/develop/include/boost/math/tools/cubic_roots.hpp)
// because it was introduced in a newer version that I don't have access to :(
template <typename Real>
std::array<Real, 3> cubic_roots(Real a, Real b, Real c, Real d)
{
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

double round(const double v, const unsigned int n);

Eigen::Matrix3d rotation_to_align(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1);

double calculate_confocal_ellipsoid_surface(
    const double beta,
    const double gamma,
    const double x,
    const double y,
    const double z
);

Eigen::Vector3d calculate_cartesian_effective_force(
    const double beta,
    const double gamma,
    const double omega,
    const double x,
    const double y,
    const double z,
    const double lambda
);

Eigen::Vector3d calculate_cartesian_effective_force(
    const double beta,
    const double gamma,
    const double omega,
    const double x,
    const double y,
    const double z,
    const double lambda,
    double& magnitude
);

}}
