#pragma once

#include "asteroid_mining/asteroid/carrier.hpp"
#include "asteroid_mining/dimensions_scaler.hpp"
#include "asteroid_mining/originator_i.hpp"
#include "asteroid_mining/progressable_i.hpp"

#include <Eigen/Dense>

namespace asteroid_mining {

class AsteroidOriginator : public AsteroidCarrier, public OriginatorI<AsteroidCarrier>, public ProgressableI
{
public:
    AsteroidOriginator(
        const double beta_,
        const double gamma_,
        const double density_,
        const double rho_A,
        const double omega_bar
    );

    static AsteroidOriginator from_dimensioned_values(
        const DimensionsScaler& dimensions_scaler,
        const double beta_bar,
        const double gamma_bar,
        const double rho_A,
        const double omega_bar
    );

    virtual const AsteroidCarrier& get_state() const override;

    virtual void progress_over(const double dt) override;

    bool is_point_within(const Eigen::Vector3d& p) const;

    Eigen::Vector3d calculate_cartesian_effective_force_at(const Eigen::Vector3d& position) const;
    Eigen::Vector3d calculate_cartesian_effective_force_at(const Eigen::Vector3d& position, double& magnitude) const;
};

}
