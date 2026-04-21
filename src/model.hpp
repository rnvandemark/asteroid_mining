#pragma once

#include "asteroid.hpp"
#include "math.hpp"
#include "progressable_i.hpp"
#include "siphon.hpp"

namespace am {

class Model : public ProgressableI
{
public:
    Model(Asteroid& asteroid_, Siphon& siphon_):
        asteroid(asteroid_),
        siphon(siphon_)
    {
    }

    virtual void progress_over(const double dt) override
    {
        siphon.progress_over(dt);
        asteroid.progress_over(dt);
    }

    const Asteroid& get_asteroid() const
    {
        return asteroid;
    }
    const Siphon& get_siphon() const
    {
        return siphon;
    }

    easy3d::vec3 calculate_asteroid_effective_potential_cartesian_partials(const easy3d::vec3& position) const
    {
        return calculate_effective_potential_cartesian_partials(
            asteroid.beta,
            asteroid.gamma,
            asteroid.omega,
            position.x,
            position.y,
            position.z,
            calculate_confocal_ellipsoid_surface(
                asteroid.beta,
                asteroid.gamma,
                position.x,
                position.y,
                position.z
            )
        );
    }

protected:
    Asteroid& asteroid;
    Siphon& siphon;
};

}
