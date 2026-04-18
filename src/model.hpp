#pragma once

#include "asteroid.hpp"
#include "siphon.hpp"

namespace am {

class Model
{
public:
    Model(Asteroid& asteroid_/*, Siphon& siphon_*/):
        asteroid(asteroid_)/*,
        siphon(siphon_)*/
    {
    }

    const Asteroid& get_asteroid() const
    {
        return asteroid;
    }

    void update(const double dt)
    {
        asteroid.rotate_over(dt);
    }

protected:
    Asteroid& asteroid;
    //Siphon& siphon;
};

}
