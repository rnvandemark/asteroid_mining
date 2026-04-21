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

protected:
    Asteroid& asteroid;
    Siphon& siphon;
};

}
