#pragma once

#include "asteroid_mining/asteroid/originator.hpp"
#include "asteroid_mining/dimensions_scaler.hpp"
#include "asteroid_mining/progressable_i.hpp"
#include "asteroid_mining/released_payload/originator.hpp"
#include "asteroid_mining/siphon/originator.hpp"

namespace asteroid_mining {

class Model : public ProgressableI
{
public:
    Model(DimensionsScaler& dimensions_scaler_, AsteroidOriginator& asteroid_, SiphonOriginator& siphon_);

    virtual void progress_over(const double dt) override;

    const DimensionsScaler& get_dimensions_scaler() const;
    const AsteroidOriginator& get_asteroid() const;
    const SiphonOriginator& get_siphon() const;
    const ReleasedPayloadOriginator& get_released_payload() const;

    void set_new_release_requested();

protected:
    void release_new_payload();

    DimensionsScaler& dimensions_scaler;

    AsteroidOriginator& asteroid;
    SiphonOriginator& siphon;

    ReleasedPayloadOriginator released_payload;

    bool new_release_requested;
};

}
