#pragma once

#include "dimensions_scaler.hpp"
#include "math.hpp"
#include "model.hpp"

#include <easy3d/algo/surface_mesh_factory.h>
#include <easy3d/algo/surface_mesh_triangulation.h>
#include <easy3d/core/mat.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/text_renderer.h>
#include <easy3d/renderer/vertex_array_object.h>
#include <easy3d/util/initializer.h>
#include <easy3d/viewer/viewer.h>

#include <chrono>
#include <memory>

namespace {
    easy3d::SurfaceMesh create_cube_mesh(const easy3d::vec3& geometric_center = easy3d::vec3(0, 0, 0))
    {
        easy3d::SurfaceMesh m;

        const double s = 0.5;
        const easy3d::vec3 min(geometric_center.x - s, geometric_center.y - s, geometric_center.z - s);
        const easy3d::vec3 max(geometric_center.x + s, geometric_center.y + s, geometric_center.z + s);

        const std::vector<easy3d::SurfaceMesh::Vertex> v = {
            m.add_vertex(easy3d::vec3(min.x, min.y, max.z)),
            m.add_vertex(easy3d::vec3(max.x, min.y, max.z)),
            m.add_vertex(easy3d::vec3(max.x, max.y, max.z)),
            m.add_vertex(easy3d::vec3(min.x, max.y, max.z)),
            m.add_vertex(easy3d::vec3(min.x, min.y, min.z)),
            m.add_vertex(easy3d::vec3(max.x, min.y, min.z)),
            m.add_vertex(easy3d::vec3(max.x, max.y, min.z)),
            m.add_vertex(easy3d::vec3(min.x, max.y, min.z)),
        };

        // Front face
        m.add_triangle(v[0], v[1], v[2]);
        m.add_triangle(v[0], v[2], v[3]);
        // Back face
        m.add_triangle(v[5], v[4], v[7]);
        m.add_triangle(v[5], v[7], v[6]);
        // Top face
        m.add_triangle(v[3], v[2], v[6]);
        m.add_triangle(v[3], v[6], v[7]);
        // Bottom face
        m.add_triangle(v[4], v[5], v[1]);
        m.add_triangle(v[4], v[1], v[0]);
        // Right face
        m.add_triangle(v[1], v[5], v[6]);
        m.add_triangle(v[1], v[6], v[2]);
        // Left face
        m.add_triangle(v[4], v[0], v[3]);
        m.add_triangle(v[4], v[3], v[7]);

        return m;
    }

    easy3d::SurfaceMesh create_arrow_mesh(
        const easy3d::vec3& geometric_center = easy3d::vec3(0, 0, 0),
        const float shaft_radius = 1.0,
        const float shaft_height = 2.0,
        const float tip_radius = 1.0,
        const float tip_height = 1.0
    )
    {
        easy3d::SurfaceMesh shaft = easy3d::SurfaceMeshFactory::cylinder(6, shaft_radius, shaft_height);

        easy3d::SurfaceMesh tip = easy3d::SurfaceMeshFactory::cone(6, tip_radius, tip_height);
        for (const auto& v : tip.vertices())
        {
            tip.position(v).z += shaft_height;
        }

        easy3d::SurfaceMesh arrow = shaft.join(tip);

        const float dx = std::max(shaft_radius, tip_radius) / 2;
        const float dy = dx;
        const float dz = (shaft_height + tip_height) / 2;
        for (const auto& v : arrow.vertices())
        {
            auto& p = arrow.position(v);
            p.x += (geometric_center.x - dx);
            p.y += (geometric_center.y - dy);
            p.z += (geometric_center.z - dz);
        }

        easy3d::SurfaceMeshTriangulation triangulator(&arrow);
        triangulator.triangulate();

        return arrow;
    }
}

namespace am {

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
        const int width = 1600,
        const int height = 1200,
        const bool full_screen = false,
        const bool resizable = true
    ) :
        easy3d::Viewer(
            title,
            4,
            3,
            2,
            full_screen,
            resizable,
            24,
            8,
            width,
            height
        ),
        time_per_second(TimePerSecond::SECONDS_ONE)
    {
        show_easy3d_logo_ = false;
        show_frame_rate_ = true;

        bind(
            [this](easy3d::Viewer*, easy3d::Model*) -> bool { this->change_time_rate_callback(false); return true; },
            nullptr,
            easy3d::Viewer::KEY_Q
        );
        bind(
            [this](easy3d::Viewer*, easy3d::Model*) -> bool { this->change_time_rate_callback(true); return true; },
            nullptr,
            easy3d::Viewer::KEY_W
        );
    }

    int get_time_rate() const
    {
        return static_cast<int>(time_per_second);
    }

protected:
    void change_time_rate_callback(const bool incremented)
    {
        switch (time_per_second)
        {
            case TimePerSecond::SECONDS_ONE:
                time_per_second = (incremented ? TimePerSecond::SECONDS_TEN : TimePerSecond::SECONDS_ONE);
                break;
            case TimePerSecond::SECONDS_TEN:
                time_per_second = (incremented ? TimePerSecond::SECONDS_THIRTY : TimePerSecond::SECONDS_ONE);
                break;
            case TimePerSecond::SECONDS_THIRTY:
                time_per_second = (incremented ? TimePerSecond::MINUTES_ONE : TimePerSecond::SECONDS_TEN);
                break;
            case TimePerSecond::MINUTES_ONE:
                time_per_second = (incremented ? TimePerSecond::MINUTES_FIVE : TimePerSecond::SECONDS_THIRTY);
                break;
            case TimePerSecond::MINUTES_FIVE:
                time_per_second = (incremented ? TimePerSecond::MINUTES_TEN : TimePerSecond::MINUTES_ONE);
                break;
            case TimePerSecond::MINUTES_TEN:
                time_per_second = (incremented ? TimePerSecond::MINUTES_THIRTY : TimePerSecond::MINUTES_FIVE);
                break;
            case TimePerSecond::MINUTES_THIRTY:
                time_per_second = (incremented ? TimePerSecond::HOURS_ONE : TimePerSecond::MINUTES_TEN);
                break;
            case TimePerSecond::HOURS_ONE:
                time_per_second = (incremented ? TimePerSecond::HOURS_FIVE : TimePerSecond::MINUTES_THIRTY);
                break;
            case TimePerSecond::HOURS_FIVE:
                time_per_second = (incremented ? TimePerSecond::HOURS_FIVE : TimePerSecond::HOURS_ONE);
                break;
        }
    }

    virtual void post_draw() override
    {
        easy3d::Viewer::post_draw();

        std::string desc = "Passage of time: ";
        switch (time_per_second)
        {
            case TimePerSecond::SECONDS_ONE:
                desc = "1 sec";
                break;
            case TimePerSecond::SECONDS_TEN:
                desc = "10 secs";
                break;
            case TimePerSecond::SECONDS_THIRTY:
                desc = "30 secs";
                break;
            case TimePerSecond::MINUTES_ONE:
                desc = "1 min";
                break;
            case TimePerSecond::MINUTES_FIVE:
                desc = "5 mins";
                break;
            case TimePerSecond::MINUTES_TEN:
                desc = "10 mins";
                break;
            case TimePerSecond::MINUTES_THIRTY:
                desc = "30 mins";
                break;
            case TimePerSecond::HOURS_ONE:
                desc = "1 hr";
                break;
            case TimePerSecond::HOURS_FIVE:
                desc = "5 hrs";
                break;
        }
        desc += " / sec";

        const float font_size = 15.0f;
        float x = 20.0 * dpi_scaling();
        float y = 2 * font_size * 1.5 * dpi_scaling();
        texter_->draw(desc, x, y, font_size, 1);
    }

    TimePerSecond time_per_second;
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
        ) :
            mesh(mesh_),
            triangle_points(),
            triangle_indices(),
            surface(std::make_shared<easy3d::TrianglesDrawable>(drawable_name))
        {
            for (const auto& v : mesh.vertices())
            {
                auto& p = mesh.position(v);
                p.x *= scale_x;
                p.y *= scale_y;
                p.z *= scale_z;
            }

            triangle_points = mesh.points();

            for (auto fh : mesh.faces())
            {
                auto circulator = mesh.vertices(fh);
                auto end = circulator;
                do {
                    triangle_indices.push_back((*circulator).idx());
                    ++circulator;
                } while (circulator != end);
            }

            surface->set_uniform_coloring(mesh_color);
            surface->update_vertex_buffer(triangle_points, true);
            surface->update_element_buffer(triangle_indices);
        }

        easy3d::SurfaceMesh mesh;
        std::vector<easy3d::vec3> triangle_points;
        std::vector<unsigned int> triangle_indices;
        std::shared_ptr<easy3d::TrianglesDrawable> surface;
    };

    struct GravityGradientMarkerMesh
    {
        GravityGradientMarkerMesh(
            const DrawableMesh& arrow_,
            const easy3d::vec3& position_
        ) :
            arrow(arrow_),
            position(position_)
        {
        }

        DrawableMesh arrow;
        const easy3d::vec3 position;
    };

public:
    View(
        const DimensionsScaler& dimensions_scaler_,
        Model& model_,
        const double gravity_gradient_shell_radius,
        const unsigned int gravity_gradient_phi_step,
        const unsigned int gravity_gradient_theta_step
    ) :
        window("Triaxial Ellipsoid Viewer"),
        dimensions_scaler(dimensions_scaler_),
        model(model_),
        asteroid_mesh(
            "asteroid",
            easy3d::SurfaceMeshFactory::icosphere(),
            easy3d::vec4(0.4, 0.4, 0, 1),
            model.get_asteroid().alpha,
            model.get_asteroid().beta,
            model.get_asteroid().gamma
        ),
        siphon_mesh(
            "siphon",
            create_cube_mesh(easy3d::vec3(0, 0, 0.5)),
            easy3d::vec4(1, 0, 0, 1),
            dimensions_scaler.get_dimensionless(
                5, DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
            ),
            dimensions_scaler.get_dimensionless(
                5, DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
            ),
            model.get_siphon().chain_length
        ),
        gravity_gradient_marker_meshes(),
        start_animation_time(),
        last_animation_time()
    {
        window.camera()->setZClippingCoefficient(100000);

        window.add_drawable(asteroid_mesh.surface);
        window.add_drawable(siphon_mesh.surface);
        for (unsigned int theta_deg = 0; theta_deg <= 360; theta_deg += gravity_gradient_theta_step)
        {
            const float theta = theta_deg * M_PI / 180;
            for (unsigned int phi_deg = 0; phi_deg <= 180; phi_deg += gravity_gradient_phi_step)
            {
                const float phi = phi_deg * M_PI / 180;
                DrawableMesh m(
                    "arrow [" + std::to_string(phi_deg) + "/" + std::to_string(theta_deg) + "]",
                    create_arrow_mesh(),
                    easy3d::vec4(0, 0, 0, 1),
                    dimensions_scaler.get_dimensionless(
                        5, DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
                    ),
                    dimensions_scaler.get_dimensionless(
                        5, DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
                    ),
                    dimensions_scaler.get_dimensionless(
                        15, DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
                    )
                );
                gravity_gradient_marker_meshes.push_back(GravityGradientMarkerMesh(
                    m,
                    easy3d::vec3(
                        gravity_gradient_shell_radius * std::sin(theta) * std::cos(phi),
                        gravity_gradient_shell_radius * std::sin(theta) * std::sin(phi),
                        gravity_gradient_shell_radius * std::cos(theta)
                    )
                ));
                window.add_drawable(m.surface);
            }
        }

        window.animation_func_= [this](easy3d::Viewer*) -> bool { return this->animation_callback(); };
        window.set_animation(true);
    }

    int run()
    {
        start_animation_time = std::chrono::system_clock::now();
        last_animation_time = start_animation_time;
        easy3d::initialize();
        return window.run();
    }

protected:
    easy3d::vec3 get_dimensioned(const easy3d::vec3& dimensionless, const DimensionsScaler::ScaleOpChain& chain) const
    {
        return easy3d::vec3(
            dimensions_scaler.get_dimensioned(dimensionless.x, chain),
            dimensions_scaler.get_dimensioned(dimensionless.y, chain),
            dimensions_scaler.get_dimensioned(dimensionless.z, chain)
        );
    }

    bool animation_callback()
    {
        const auto now = std::chrono::system_clock::now();
        const double dt = std::chrono::duration<double>(now - last_animation_time).count();
        model.progress_over(window.get_time_rate() * dimensions_scaler.get_dimensionless(
            dt,
            DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::TIME)
        ));
        const bool rc = render_model();
        last_animation_time = now;
        return rc;
    }

    bool render_model()
    {
        const auto& asteroid = model.get_asteroid();
        const auto& siphon = model.get_siphon();

        const easy3d::Quat<float> q__universe__asteroid(easy3d::Vec<3, float>(0, 0, 1), asteroid.get_rotation());
        const easy3d::Quat<float> q__asteroid__universe = q__universe__asteroid.inverse();
        const easy3d::Mat4<float> T__universe__siphon_base = easy3d::Mat4<float>::rotation(q__universe__asteroid)
            * easy3d::Mat4<float>::rotation(easy3d::Quat<float>(easy3d::Vec<3, float>(0, 0, 1), siphon.anchor_point_polar_angle))
            * easy3d::Mat4<float>::translation(siphon.anchor_point_polar_radius, 0.0, 0.0)
            * easy3d::Mat4<float>::rotation(easy3d::Quat<float>(easy3d::Vec<3, float>(0, 1, 0), M_PI/2))
        ;

        {
            easy3d::vec3* const vertex_buffer = reinterpret_cast<easy3d::vec3*>(easy3d::VertexArrayObject::map_buffer(
                GL_ARRAY_BUFFER, asteroid_mesh.surface->vertex_buffer(), GL_WRITE_ONLY
            ));
            if (!vertex_buffer)
            {
                return false;
            }

            // Rotate the asteroid's mesh
            for (std::size_t i = 0; i < asteroid_mesh.triangle_points.size(); i++)
            {
                const auto p = get_dimensioned(
                    q__universe__asteroid.rotate(asteroid_mesh.triangle_points[i]),
                    DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
                );
                vertex_buffer[i].x = p.x;
                vertex_buffer[i].y = p.y;
                vertex_buffer[i].z = p.z;
            }

            easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, asteroid_mesh.surface->vertex_buffer());
        }

        {
            easy3d::vec3* const vertex_buffer = reinterpret_cast<easy3d::vec3*>(easy3d::VertexArrayObject::map_buffer(
                GL_ARRAY_BUFFER, siphon_mesh.surface->vertex_buffer(), GL_WRITE_ONLY
            ));
            if (!vertex_buffer)
            {
                return false;
            }

            // Transform the siphon's mesh
            for (std::size_t i = 0; i < siphon_mesh.triangle_points.size(); i++)
            {
                const auto p = get_dimensioned(
                    T__universe__siphon_base * siphon_mesh.triangle_points[i],
                    DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
                );
                vertex_buffer[i].x = p.x;
                vertex_buffer[i].y = p.y;
                vertex_buffer[i].z = p.z;
            }

            easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, siphon_mesh.surface->vertex_buffer());
        }

        {
            std::vector<easy3d::vec3> gravity_gradients(gravity_gradient_marker_meshes.size());
            std::vector<float> gravity_gradient_magnitudes(gravity_gradient_marker_meshes.size());

            for (std::size_t i = 0; i < gravity_gradient_marker_meshes.size(); i++)
            {
                const auto marker_p__asteroid = q__asteroid__universe.rotate(gravity_gradient_marker_meshes[i].position);
                gravity_gradients[i] = calculate_effective_potential_cartesian_partials(
                    asteroid.beta,
                    asteroid.gamma,
                    asteroid.omega,
                    marker_p__asteroid.x,
                    marker_p__asteroid.y,
                    marker_p__asteroid.z,
                    calculate_confocal_ellipsoid_surface(
                        asteroid.beta,
                        asteroid.gamma,
                        marker_p__asteroid.x,
                        marker_p__asteroid.y,
                        marker_p__asteroid.z
                    )
                );
                gravity_gradient_magnitudes[i] = gravity_gradients[i].length();
            }

            const auto min_mag_iter = std::min_element(gravity_gradient_magnitudes.cbegin(), gravity_gradient_magnitudes.cend());
            const auto max_mag_iter = std::max_element(gravity_gradient_magnitudes.cbegin(), gravity_gradient_magnitudes.cend());
            const float min_mag = *min_mag_iter;
            const float max_mag = *max_mag_iter;

            for (std::size_t i = 0; i < gravity_gradient_marker_meshes.size(); i++)
            {
                const auto& mesh = gravity_gradient_marker_meshes[i].arrow;
                const auto T__marker__CoG = easy3d::Mat4<float>::translation(gravity_gradient_marker_meshes[i].position)
                    * easy3d::Mat4<float>(rotation_to_align(easy3d::vec3(0, 0, 1), q__universe__asteroid.rotate(gravity_gradients[i])))
                ;
                const float relative_norm = (gravity_gradient_magnitudes[i] - min_mag) / (max_mag - min_mag);

                easy3d::vec3* const vertex_buffer = reinterpret_cast<easy3d::vec3*>(easy3d::VertexArrayObject::map_buffer(
                    GL_ARRAY_BUFFER, mesh.surface->vertex_buffer(), GL_WRITE_ONLY
                ));
                if (!vertex_buffer)
                {
                    return false;
                }

                // For each of these markers, calculate the gravity gradient at these locations
                for (std::size_t j = 0; j < mesh.triangle_points.size(); j++)
                {
                    const auto p = get_dimensioned(
                        T__marker__CoG * mesh.triangle_points[j],
                        DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
                    );
                    vertex_buffer[j].x = p.x;
                    vertex_buffer[j].y = p.y;
                    vertex_buffer[j].z = p.z;
                }

                // Color it based on how relatively strong the gradient is
                mesh.surface->set_uniform_coloring(easy3d::vec4(relative_norm, 0, 0, 1));

                easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, mesh.surface->vertex_buffer());
            }
        }

        window.update();
        return true;
    }

    AsteroidMiningViewer window;

    const DimensionsScaler& dimensions_scaler;
    Model& model;

    DrawableMesh asteroid_mesh;
    DrawableMesh siphon_mesh;
    std::vector<GravityGradientMarkerMesh> gravity_gradient_marker_meshes;

    std::chrono::time_point<std::chrono::system_clock> start_animation_time;
    std::chrono::time_point<std::chrono::system_clock> last_animation_time;
};

}
