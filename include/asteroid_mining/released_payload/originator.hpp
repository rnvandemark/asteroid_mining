#pragma once

#include "asteroid_mining/asteroid/originator.hpp"
#include "asteroid_mining/originator_i.hpp"
#include "asteroid_mining/progressable_i.hpp"
#include "asteroid_mining/released_payload/carrier.hpp"

namespace asteroid_mining {

class ReleasedPayloadOriginator : public ReleasedPayloadCarrier, public OriginatorI<ReleasedPayloadCarrier>, public ProgressableI
{
public:
    ReleasedPayloadOriginator(const AsteroidOriginator& asteroid_);

    virtual const ReleasedPayloadCarrier& get_state() const override;

    virtual void progress_over(const double dt) override;

    void release(
        const double mass_,
        const Eigen::Vector3d& position_,
        const Eigen::Vector3d& velocity_,
        const Eigen::Vector3d& acceleration_
    );

protected:
    const AsteroidOriginator& asteroid;
};

}
