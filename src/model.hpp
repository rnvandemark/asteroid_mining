#pragma once

#include "asteroid.hpp"
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

    const Asteroid& get_asteroid() const
    {
        return asteroid;
    }
    const Siphon& get_siphon() const
    {
        return siphon;
    }

    virtual void progress_over(const double dt) override
    {
        siphon.progress_over(dt);
        asteroid.progress_over(dt);
    }

protected:
    Asteroid& asteroid;
    Siphon& siphon;
};

}
