#pragma once

#include "asteroid.hpp"
#include "progressable_i.hpp"
#include "released_payload.hpp"
#include "siphon.hpp"

namespace am {

class Model : public ProgressableI
{
public:
    Model(Asteroid& asteroid_, Siphon& siphon_):
        asteroid(asteroid_),
        siphon(siphon_),
        released_payload(asteroid),
        new_release_requested(false)
    {
    }

    virtual void progress_over(const double dt) override
    {
        if (new_release_requested)
        {
            release_new_payload();
        }
        siphon.progress_over(dt);
        asteroid.progress_over(dt);
        released_payload.progress_over(dt);
    }

    const Asteroid& get_asteroid() const
    {
        return asteroid;
    }
    const Siphon& get_siphon() const
    {
        return siphon;
    }
    const ReleasedPayload& get_released_payload() const
    {
        return released_payload;
    }

    void set_new_release_requested()
    {
        new_release_requested = true;
    }

protected:
    void release_new_payload()
    {
        const double theta = siphon.get_siphon_angular_position();
        const double v_theta = siphon.chain_length * siphon.get_siphon_angular_velocity();
        released_payload.release(
            siphon.get_cs_payload_mass(),
            siphon.get_position_in_asteroid_frame(siphon.chain_length),
            easy3d::vec3(-v_theta * std::sin(theta), v_theta * std::cos(theta), 0),
            easy3d::vec3(0, 0, 0)
        );
        siphon.clear_cs_payload_mass();
        new_release_requested = false;
    }

    Asteroid& asteroid;
    Siphon& siphon;

    ReleasedPayload released_payload;

    bool new_release_requested;
};

}
