#pragma once

namespace am {

class Siphon
{
public:
    Siphon(
        const unsigned int n_,
        const double chain_length_,
        const double bucket_mass_,
        const double payload_mass_,
        const double cs_dry_mass_,
        const double anchor_point_angle_
    ):
        n(n_),
        chain_length(chain_length_),
        bucket_mass(bucket_mass_),
        payload_mass(payload_mass_),
        lifting_side_mass(bucket_mass + payload_mass),
        descending_side_mass(bucket_mass),
        cs_dry_mass(cs_dry_mass_),
        anchor_point_angle(anchor_point_angle_),
        cs_total_mass(cs_dry_mass)
    {
    }

protected:
    // The number of buckets on each side of the chain, so there are 2n total.
    const unsigned int n;

    const double chain_length;
    const double bucket_mass;
    const double payload_mass;
    const double lifting_side_mass;
    const double descending_side_mass;

    const double cs_dry_mass;

    const double anchor_point_angle;

    double cs_total_mass;
};

}
