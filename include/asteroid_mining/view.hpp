#pragma once

#include "asteroid_mining/model.hpp"

#include <easy3d/core/surface_mesh.h>
#include <easy3d/viewer/viewer.h>

#include <chrono>
#include <memory>

namespace asteroid_mining {

class AsteroidMiningViewer : public easy3d::Viewer
{
public:
    enum class TimePerSecond : int
    {
        SECONDS_ONE = 1,
        SECONDS_TEN = 10,
        SECONDS_THIRTY = 30,
        MINUTES_ONE = 60,
        MINUTES_FIVE = 300,
        MINUTES_TEN = 600,
        MINUTES_THIRTY = 1800,
        HOURS_ONE = 3600,
        HOURS_FIVE = 18000,
    };

    AsteroidMiningViewer(
        const std::string& title,
        Model& model_,
        const easy3d::vec4& bg_color = easy3d::vec4(0, 0, 0, 1),
        const int width = 1600,
        const int height = 1200,
        const bool full_screen = false,
        const bool resizable = true
    );

    int get_time_rate() const;

    bool showing_effective_forces() const;

    double get_effective_force_markers_radius() const;

    bool corotating_camera_with_asteroid() const;

protected:
    void change_time_rate_callback(const bool incremented);

    void change_effective_force_markers_radius_callback(const bool incremented, const bool accelerated);

    virtual void post_draw() override;

    Model& model;

    TimePerSecond time_per_second;

    bool show_effective_forces;
    double effective_force_markers_radius;

    bool corotate_camera_with_asteroid;
};

class View
{
protected:
    class DrawableMesh
    {
    public:
        DrawableMesh(
            const std::string& drawable_name,
            const easy3d::SurfaceMesh& mesh_,
            const easy3d::vec4& mesh_color,
            const double scale_x = 1.0,
            const double scale_y = 1.0,
            const double scale_z = 1.0
        );

        easy3d::SurfaceMesh mesh;
        std::vector<easy3d::vec3> triangle_points;
        std::vector<unsigned int> triangle_indices;
        std::shared_ptr<easy3d::TrianglesDrawable> surface;
    };

public:
    View(
        Model& model_,
        const double siphon_width_,
        const unsigned int num_latitudinal_effective_force_marker_rings_,
        const unsigned int num_longitudinal_effective_force_markers_
    );

    int run();

protected:
    easy3d::vec3 get_dimensioned(const easy3d::vec3& dimensionless, const DimensionsScaler::ScaleOpChain& chain) const;

    bool animation_callback();

    bool render_model();

    AsteroidMiningViewer window;

    Model& model;

    const double siphon_width;
    const unsigned int num_latitudinal_effective_force_marker_rings;
    const unsigned int num_longitudinal_effective_force_markers;

    DrawableMesh asteroid_mesh;
    DrawableMesh siphon_mesh;
    DrawableMesh collecting_satellite_mesh;
    DrawableMesh released_payload_mesh;
    std::vector<DrawableMesh> siphon_mass_meshes;
    std::vector<std::vector<DrawableMesh>> effective_force_marker_meshes;

    std::chrono::time_point<std::chrono::system_clock> start_animation_time;
    std::chrono::time_point<std::chrono::system_clock> last_animation_time;
};

}
