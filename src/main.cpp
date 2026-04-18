#include "asteroid.hpp"
#include "dimensions_scaler.hpp"
#include "model.hpp"
#include "siphon.hpp"
#include "view.hpp"

#include <easy3d/util/initializer.h>

int main(int argc, char** argv)
{
    const double alpha_bar = 565.0;  // m
    const double beta_bar = 535.0;  // m
    const double gamma_bar = 508.0;  // m
    const double rho_bar = 2500.0;  // kg/m^3
    const double omega_bar = 0.0004062678046;  // rad/s

    const double bucket_mass = 50.0;
    const double payload_mass = 200.0;

    am::DimensionsScaler dimensions_scaler(alpha_bar, bucket_mass + payload_mass, 1 / omega_bar);

    am::Asteroid asteroid = am::Asteroid::from_dimensioned_values(
        dimensions_scaler, beta_bar, gamma_bar, rho_bar
    );
    am::Model model(asteroid);

    easy3d::initialize();
    return am::View(dimensions_scaler, model).run();
}
