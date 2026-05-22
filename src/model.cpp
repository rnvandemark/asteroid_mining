#include "asteroid_mining/model.hpp"

namespace asteroid_mining {

Model::Model(DimensionsScaler& dimensions_scaler_, AsteroidOriginator& asteroid_, SiphonOriginator& siphon_):
    dimensions_scaler(dimensions_scaler_),
    asteroid(asteroid_),
    siphon(siphon_),
    released_payload(asteroid),
    new_release_requested(false)
{
}

void Model::progress_over(const double dt)
{
    if (new_release_requested)
    {
        release_new_payload();
    }
    siphon.progress_over(dt);
    asteroid.progress_over(dt);
    released_payload.progress_over(dt);
}

const DimensionsScaler& Model::get_dimensions_scaler() const
{
    return dimensions_scaler;
}
const AsteroidOriginator& Model::get_asteroid() const
{
    return asteroid;
}
const SiphonOriginator& Model::get_siphon() const
{
    return siphon;
}
const ReleasedPayloadOriginator& Model::get_released_payload() const
{
    return released_payload;
}

void Model::set_new_release_requested()
{
    new_release_requested = true;
}

void Model::release_new_payload()
{
    const double theta = siphon.get_siphon_angular_position();
    const double v_theta = siphon.get_chain_length() * siphon.get_siphon_angular_velocity();
    released_payload.release(
        siphon.get_cs_payload_mass(),
        siphon.get_position_in_asteroid_frame(siphon.get_chain_length()),
        Eigen::Vector3d{-v_theta * std::sin(theta), v_theta * std::cos(theta), 0},
        Eigen::Vector3d{0, 0, 0}
    );
    siphon.clear_cs_payload_mass();
    new_release_requested = false;
}

}
