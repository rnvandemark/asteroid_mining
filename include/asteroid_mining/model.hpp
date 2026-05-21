#pragma once

#include "asteroid_mining/asteroid.hpp"
#include "asteroid_mining/progressable_i.hpp"
#include "asteroid_mining/released_payload.hpp"
#include "asteroid_mining/siphon.hpp"

namespace asteroid_mining {

class Model : public ProgressableI
{
public:
    Model(Asteroid& asteroid_, Siphon& siphon_);

    virtual void progress_over(const double dt) override;

    const Asteroid& get_asteroid() const;
    const Siphon& get_siphon() const;
    const ReleasedPayload& get_released_payload() const;

    void set_new_release_requested();

protected:
    void release_new_payload();

    Asteroid& asteroid;
    Siphon& siphon;

    ReleasedPayload released_payload;

    bool new_release_requested;
};

}
