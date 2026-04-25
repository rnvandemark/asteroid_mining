#pragma once

#include "dimensions_scaler.hpp"
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

#include <numeric>
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
        const DimensionsScaler& dimensions_scaler_,
        Model& model_,
        const easy3d::vec4& bg_color = easy3d::vec4(0, 0, 0, 1),
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
        dimensions_scaler(dimensions_scaler_),
        model(model_),
        time_per_second(TimePerSecond::SECONDS_ONE),
        show_effective_forces(true),
        effective_force_markers_radius(1.5),
        corotate_camera_with_asteroid(false)
    {
        show_easy3d_logo_ = false;
        show_frame_rate_ = true;
        set_background_color(bg_color);

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

        bind(
            [this](easy3d::Viewer*, easy3d::Model*) -> bool { show_effective_forces = (!show_effective_forces); return true; },
            nullptr,
            easy3d::Viewer::KEY_G
        );

        bind(
            [this](easy3d::Viewer*, easy3d::Model*) -> bool { corotate_camera_with_asteroid = (!corotate_camera_with_asteroid); return true; },
            nullptr,
            easy3d::Viewer::KEY_L
        );

        bind(
            [this](easy3d::Viewer*, easy3d::Model*) -> bool { change_effective_force_markers_radius_callback(false, false); return true; },
            nullptr,
            easy3d::Viewer::KEY_O
        );
        bind(
            [this](easy3d::Viewer*, easy3d::Model*) -> bool { change_effective_force_markers_radius_callback(true, false); return true; },
            nullptr,
            easy3d::Viewer::KEY_P
        );
        bind(
            [this](easy3d::Viewer*, easy3d::Model*) -> bool { change_effective_force_markers_radius_callback(false, true); return true; },
            nullptr,
            easy3d::Viewer::KEY_O,
            easy3d::Viewer::MODIF_SHIFT
        );
        bind(
            [this](easy3d::Viewer*, easy3d::Model*) -> bool { change_effective_force_markers_radius_callback(true, true); return true; },
            nullptr,
            easy3d::Viewer::KEY_P,
            easy3d::Viewer::MODIF_SHIFT
        );

        bind(
            [this](easy3d::Viewer*, easy3d::Model*) -> bool { model.set_new_release_requested(); return true; },
            nullptr,
            easy3d::Viewer::KEY_R
        );
    }

    int get_time_rate() const
    {
        return static_cast<int>(time_per_second);
    }

    bool showing_effective_forces() const
    {
        return show_effective_forces;
    }

    double get_effective_force_markers_radius() const
    {
        return effective_force_markers_radius;
    }

    bool corotating_camera_with_asteroid() const
    {
        return corotate_camera_with_asteroid;
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

    void change_effective_force_markers_radius_callback(const bool incremented, const bool accelerated)
    {
        effective_force_markers_radius = std::max(
            effective_force_markers_radius + ((incremented ? 1 : -1) * (accelerated ? 0.1 : 0.01)),
            1.01
        );
    }

    virtual void post_draw() override
    {
        easy3d::Viewer::post_draw();

        const float font_size = 15.0f;
        float x = 20.0f * dpi_scaling();
        float y = 0.0f;

        // DISCLAIMER: partially copy and pasted from easy3d::Viewer so I can
        // rewrite stuff and account for the background color
        const easy3d::vec3 font_color(1-background_color().x, 1-background_color().y, 1-background_color().z);

        if (show_easy3d_logo_)
        {
            y += font_size * 1.5f * dpi_scaling();
            texter_->draw("Easy3D", x, y, font_size, 0, font_color);
        }

        if (show_frame_rate_)
        {
            y += font_size * 1.5f * dpi_scaling();
            texter_->draw(framerate_, x, y, font_size, 1, font_color);
        }

        if (!hint_.empty())
        {
            x += font_size * dpi_scaling();
            y += font_size * 1.5f * dpi_scaling();
            texter_->draw(hint_, x, y, font_size, easy3d::TextRenderer::ALIGN_LEFT, 1, font_color);
        }

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

        y += font_size * 1.5f * dpi_scaling();
        texter_->draw(desc, x, y, font_size, 1, font_color);

        y += font_size * 1.5f * dpi_scaling();
        texter_->draw(
            "Harvested material: " + std::to_string(round(dimensions_scaler.get_dimensioned(
                model.get_siphon().get_cs_payload_mass(),
                DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::MASS)
            ), 3)) + " kg",
            x, y, font_size, 1, font_color
        );

        y += font_size * 1.5f * dpi_scaling();
        texter_->draw(
            "Time elapsed between each mass: " + std::to_string(round(dimensions_scaler.get_dimensioned(
                model.get_siphon().get_time_elapsed_last_mass_to_reach_cs() / 60,
                DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::TIME)
            ), 3)) + " min",
            x, y, font_size, 1, font_color
        );

        if (model.get_released_payload().is_active())
        {
            const auto& released_payload_p__asteroid = model.get_released_payload().get_position();
            const auto& released_payload_v__asteroid = model.get_released_payload().get_velocity();

            const easy3d::Vec<3, float> axis_of_rotation(0, 0, 1);
            const auto released_payload_p__universe = easy3d::Quat<float>(
                axis_of_rotation, model.get_asteroid().get_rotation()
            ).rotate(released_payload_p__asteroid);
            const auto released_payload_v__universe = released_payload_v__asteroid + easy3d::cross(
                axis_of_rotation, released_payload_p__asteroid
            );

            const auto stringify_p = [this] (const easy3d::vec3& p__frame) -> std::string
            {
                const auto chain = DimensionsScaler::ScaleOpChain()
                    * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
                ;
                return "-- position: [" + std::to_string(round(dimensions_scaler.get_dimensioned(p__frame.x, chain) / 1000, 3))
                    + ", " + std::to_string(round(dimensions_scaler.get_dimensioned(p__frame.y, chain) / 1000, 3))
                    + ", " + std::to_string(round(dimensions_scaler.get_dimensioned(p__frame.z, chain) / 1000, 3)) + "] km"
                ;
            };

            const auto stringify_v = [this] (const easy3d::vec3& v__frame) -> std::string
            {
                const double norm = v__frame.norm();
                return "-- velocity: " + std::to_string(round(dimensions_scaler.get_dimensioned(
                        norm,
                        DimensionsScaler::ScaleOpChain()
                            * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
                            / DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::TIME)
                    ), 3))
                    + " m/s [" + std::to_string(round(v__frame.x / norm, 3))
                    + ", " + std::to_string(round(v__frame.y / norm, 3))
                    + ", " + std::to_string(round(v__frame.z / norm, 3)) + "]"
                ;
            };

            y += font_size * 1.5f * dpi_scaling();
            texter_->draw("Released payload:", x, y, font_size, 1, font_color);

            y += font_size * 1.5f * dpi_scaling();
            texter_->draw("- asteroid frame:", x, y, font_size, 1, font_color);

            y += font_size * 1.5f * dpi_scaling();
            texter_->draw(stringify_p(released_payload_p__asteroid), x, y, font_size, 1, font_color);

            y += font_size * 1.5f * dpi_scaling();
            texter_->draw(stringify_v(released_payload_v__asteroid), x, y, font_size, 1, font_color);

            y += font_size * 1.5f * dpi_scaling();
            texter_->draw("- universe frame:", x, y, font_size, 1, font_color);

            y += font_size * 1.5f * dpi_scaling();
            texter_->draw(stringify_p(released_payload_p__universe), x, y, font_size, 1, font_color);

            y += font_size * 1.5f * dpi_scaling();
            texter_->draw(stringify_v(released_payload_v__universe), x, y, font_size, 1, font_color);
        }
    }

    const DimensionsScaler& dimensions_scaler;
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

public:
    View(
        const DimensionsScaler& dimensions_scaler_,
        Model& model_,
        const double siphon_width_,
        const unsigned int num_latitudinal_effective_force_marker_rings_,
        const unsigned int num_longitudinal_effective_force_markers_
    ) :
        window("Triaxial Ellipsoid Viewer", dimensions_scaler_, model_),
        dimensions_scaler(dimensions_scaler_),
        model(model_),
        siphon_width(dimensions_scaler.get_dimensionless(
            siphon_width_, DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
        )),
        num_latitudinal_effective_force_marker_rings(num_latitudinal_effective_force_marker_rings_),
        num_longitudinal_effective_force_markers(num_longitudinal_effective_force_markers_),
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
            easy3d::vec4(0, 0, 1, 1),
            siphon_width,
            siphon_width,
            model.get_siphon().chain_length
        ),
        collecting_satellite_mesh(
            "collecting satellite",
            create_cube_mesh(easy3d::vec3(0, 0, 0.5)),
            easy3d::vec4(1, 0, 1, 1),
            2 * siphon_width,
            2 * siphon_width,
            2 * siphon_width
        ),
        released_payload_mesh(
            "released payload",
            easy3d::SurfaceMeshFactory::icosphere(),
            easy3d::vec4(0, 0, 0, 1),
            siphon_width,
            siphon_width,
            siphon_width
        ),
        siphon_mass_meshes(),
        effective_force_marker_meshes(),
        start_animation_time(),
        last_animation_time()
    {
        window.camera()->setZClippingCoefficient(100000);

        window.add_drawable(asteroid_mesh.surface);
        window.add_drawable(siphon_mesh.surface);
        window.add_drawable(collecting_satellite_mesh.surface);
        window.add_drawable(released_payload_mesh.surface);

        for (std::size_t i = 0; i < 2 * model.get_siphon().n; i++)
        {
            const auto siphon_mass_mesh = DrawableMesh(
                "siphon mass/" + std::to_string(i),
                create_cube_mesh(),
                easy3d::vec4(0, 0, 0, 1),
                dimensions_scaler.get_dimensionless(
                    5, DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
                ),
                dimensions_scaler.get_dimensionless(
                    5, DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
                ),
                dimensions_scaler.get_dimensionless(
                    5, DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
                )
            );
            siphon_mass_meshes.push_back(siphon_mass_mesh);
            window.add_drawable(siphon_mass_mesh.surface);
        }

        effective_force_marker_meshes.resize(num_latitudinal_effective_force_marker_rings);
        for (unsigned int latitude = 0; latitude < num_latitudinal_effective_force_marker_rings; latitude++)
        {
            for (unsigned int longitude = 0; longitude < num_longitudinal_effective_force_markers; longitude++)
            {
                const auto effective_force_marker_mesh = DrawableMesh(
                    "effective force marker/" + std::to_string(latitude) + "/" + std::to_string(longitude),
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
                effective_force_marker_meshes[latitude].push_back(effective_force_marker_mesh);
                window.add_drawable(effective_force_marker_mesh.surface);
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
        const auto& released_payload = model.get_released_payload();

        const easy3d::Quat<float> q__universe__asteroid(easy3d::Vec<3, float>(0, 0, 1), asteroid.get_rotation());
        const easy3d::Quat<float> q__asteroid__universe = q__universe__asteroid.inverse();
        const easy3d::Mat4<float> T__universe__siphon_base = easy3d::Mat4<float>::rotation(q__universe__asteroid)
            * easy3d::Mat4<float>::rotation(easy3d::Quat<float>(easy3d::Vec<3, float>(0, 0, 1), siphon.anchor_point_polar_angle))
            * easy3d::Mat4<float>::translation(siphon.anchor_point_polar_radius, 0.0, 0.0)
            * easy3d::Mat4<float>::rotation(easy3d::Quat<float>(easy3d::Vec<3, float>(0, 1, 0), M_PI/2))
            * easy3d::Mat4<float>::rotation(easy3d::Quat<float>(easy3d::Vec<3, float>(-1, 0, 0), siphon.get_siphon_angular_position()))
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
            easy3d::vec3* const vertex_buffer = reinterpret_cast<easy3d::vec3*>(easy3d::VertexArrayObject::map_buffer(
                GL_ARRAY_BUFFER, collecting_satellite_mesh.surface->vertex_buffer(), GL_WRITE_ONLY
            ));
            if (!vertex_buffer)
            {
                return false;
            }

            // Transform the collecting satellite's mesh
            const easy3d::Mat4<float> T__siphon_base__collecting_satellite = easy3d::Mat4<float>::translation(0, 0, siphon.chain_length);
            for (std::size_t i = 0; i < collecting_satellite_mesh.triangle_points.size(); i++)
            {
                const auto p = get_dimensioned(
                    T__universe__siphon_base * T__siphon_base__collecting_satellite * collecting_satellite_mesh.triangle_points[i],
                    DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
                );
                vertex_buffer[i].x = p.x;
                vertex_buffer[i].y = p.y;
                vertex_buffer[i].z = p.z;
            }

            easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, collecting_satellite_mesh.surface->vertex_buffer());
        }

        {
            std::vector<std::array<double, 3>> effective_forces(siphon_mass_meshes.size());
            std::vector<double> effective_force_magnitudes(siphon_mass_meshes.size());
            std::vector<bool> effective_force_is_positives(effective_force_marker_meshes.size());

            const double net_siphon_chain_angle = siphon.anchor_point_polar_angle + siphon.get_siphon_angular_position();
            const easy3d::vec2 net_siphon_direction(std::cos(net_siphon_chain_angle), std::sin(net_siphon_chain_angle));
            for (std::size_t i = 0; i < siphon_mass_meshes.size(); i++)
            {
                effective_forces[i] = siphon.calculate_cartesian_effective_force_on_chain_at(siphon.get_mass_position(i), effective_force_magnitudes[i]);
                effective_force_is_positives[i] = std::acos(
                    easy3d::dot(net_siphon_direction, easy3d::vec2(effective_forces[i][0], effective_forces[i][1])) / effective_force_magnitudes[i]
                ) < (M_PI/2);
            }

            const double min_mag = *std::min_element(effective_force_magnitudes.cbegin(), effective_force_magnitudes.cend());
            const double max_mag = *std::max_element(effective_force_magnitudes.cbegin(), effective_force_magnitudes.cend());

            for (std::size_t i = 0; i < siphon_mass_meshes.size(); i++)
            {
                easy3d::vec3* const vertex_buffer = reinterpret_cast<easy3d::vec3*>(easy3d::VertexArrayObject::map_buffer(
                    GL_ARRAY_BUFFER, siphon_mass_meshes[i].surface->vertex_buffer(), GL_WRITE_ONLY
                ));
                if (!vertex_buffer)
                {
                    return false;
                }

                const easy3d::Mat4<float> T__siphon_base__siphon_mass = easy3d::Mat4<float>::translation(
                    0, siphon.get_mass_is_lifting(i) ? -siphon_width : siphon_width, siphon.get_mass_position(i)
                );
                const double relative_mag = (effective_force_magnitudes[i] - min_mag) / (max_mag - min_mag);

                // Transform the mesh of this mass on the siphon's chain
                for (std::size_t j = 0; j < siphon_mass_meshes[i].triangle_points.size(); j++)
                {
                    const auto p = get_dimensioned(
                        T__universe__siphon_base * T__siphon_base__siphon_mass * siphon_mass_meshes[i].triangle_points[j],
                        DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
                    );
                    vertex_buffer[j].x = p.x;
                    vertex_buffer[j].y = p.y;
                    vertex_buffer[j].z = p.z;
                }

                // Color it based on how relatively strong and which direction
                // the effective force is
                siphon_mass_meshes[i].surface->set_uniform_coloring(easy3d::vec4(
                    effective_force_is_positives[i] ? 1-relative_mag : 1,
                    effective_force_is_positives[i] ? 1 : 1-relative_mag,
                    1-relative_mag,
                    1
                ));

                easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, siphon_mass_meshes[i].surface->vertex_buffer());
            }
        }

        if (window.showing_effective_forces())
        {
            const double effective_forces_radius = window.get_effective_force_markers_radius();
            std::vector<std::vector<easy3d::vec3>> effective_force_marker_positions(
                num_latitudinal_effective_force_marker_rings,
                std::vector<easy3d::vec3>(num_longitudinal_effective_force_markers)
            );
            std::vector<std::vector<std::array<double, 3>>> effective_forces(
                num_latitudinal_effective_force_marker_rings,
                std::vector<std::array<double, 3>>(num_longitudinal_effective_force_markers)
            );
            std::vector<std::vector<double>> effective_force_magnitudes(
                num_latitudinal_effective_force_marker_rings,
                std::vector<double>(num_longitudinal_effective_force_markers)
            );

            // For each of these markers, calculate the effective force at
            // these locations
            for (unsigned int latitude = 0; latitude < num_latitudinal_effective_force_marker_rings; latitude++)
            {
                const double theta = (latitude + 1) * M_PI / num_latitudinal_effective_force_marker_rings;
                for (unsigned int longitude = 0; longitude < num_longitudinal_effective_force_markers; longitude++)
                {
                    const double phi = longitude * 2 * M_PI / num_longitudinal_effective_force_markers;
                    effective_force_marker_positions[latitude][longitude].x = effective_forces_radius * std::sin(theta) * std::cos(phi);
                    effective_force_marker_positions[latitude][longitude].y = effective_forces_radius * std::sin(theta) * std::sin(phi);
                    effective_force_marker_positions[latitude][longitude].z = effective_forces_radius * std::cos(theta);
                    effective_forces[latitude][longitude] = asteroid.calculate_cartesian_effective_force_at(
                        q__asteroid__universe.rotate(effective_force_marker_positions[latitude][longitude]),
                        effective_force_magnitudes[latitude][longitude]
                    );
                }
            }

            const std::vector<double> flattened_effective_force_magnitudes = std::accumulate(
                effective_force_magnitudes.cbegin(),
                effective_force_magnitudes.cend(),
                std::vector<double>(),
                [] (std::vector<double>& a, const std::vector<double>& b)
                {
                    a.insert(a.end(), b.cbegin(), b.cend());
                    return a;
                }
            );
            const double min_mag = *std::min_element(flattened_effective_force_magnitudes.cbegin(), flattened_effective_force_magnitudes.cend());
            const double max_mag = *std::max_element(flattened_effective_force_magnitudes.cbegin(), flattened_effective_force_magnitudes.cend());

            for (unsigned int latitude = 0; latitude < num_latitudinal_effective_force_marker_rings; latitude++)
            {
                for (unsigned int longitude = 0; longitude < num_longitudinal_effective_force_markers; longitude++)
                {
                    const auto& mesh = effective_force_marker_meshes[latitude][longitude];
                    const auto T__marker__CoG = easy3d::Mat4<float>::translation(effective_force_marker_positions[latitude][longitude])
                        * easy3d::Mat4<float>(rotation_to_align(
                            easy3d::vec3(0, 0, 1),
                            q__universe__asteroid.rotate(easy3d::vec3(
                                effective_forces[latitude][longitude][0],
                                effective_forces[latitude][longitude][1],
                                effective_forces[latitude][longitude][2]
                            ))
                        ))
                    ;
                    const double relative_mag = (effective_force_magnitudes[latitude][longitude] - min_mag) / (max_mag - min_mag);

                    easy3d::vec3* const vertex_buffer = reinterpret_cast<easy3d::vec3*>(easy3d::VertexArrayObject::map_buffer(
                        GL_ARRAY_BUFFER, mesh.surface->vertex_buffer(), GL_WRITE_ONLY
                    ));
                    if (!vertex_buffer)
                    {
                        return false;
                    }

                    // Transform the mesh of this effective force marker
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

                    // Color it based on how relatively strong the effective
                    // force is
                    mesh.surface->set_uniform_coloring(easy3d::vec4(1, 1-relative_mag, 1-relative_mag, 1));

                    easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, mesh.surface->vertex_buffer());
                }
            }
        }
        else
        {
            for (unsigned int latitude = 0; latitude < num_latitudinal_effective_force_marker_rings; latitude++)
            {
                for (unsigned int longitude = 0; longitude < num_longitudinal_effective_force_markers; longitude++)
                {
                    const auto& mesh = effective_force_marker_meshes[latitude][longitude];

                    easy3d::vec3* const vertex_buffer = reinterpret_cast<easy3d::vec3*>(easy3d::VertexArrayObject::map_buffer(
                        GL_ARRAY_BUFFER, mesh.surface->vertex_buffer(), GL_WRITE_ONLY
                    ));
                    if (!vertex_buffer)
                    {
                        return false;
                    }

                    // Put all of the markers inside of the asteroid to effectively
                    // hide them
                    for (std::size_t j = 0; j < mesh.triangle_points.size(); j++)
                    {
                        vertex_buffer[j].x = 0;
                        vertex_buffer[j].y = 0;
                        vertex_buffer[j].z = 0;
                    }

                    mesh.surface->set_uniform_coloring(easy3d::vec4(0, 0, 0, 1));

                    easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, mesh.surface->vertex_buffer());
                }
            }
        }

        if (released_payload.is_active())
        {
            easy3d::vec3* const vertex_buffer = reinterpret_cast<easy3d::vec3*>(easy3d::VertexArrayObject::map_buffer(
                GL_ARRAY_BUFFER, released_payload_mesh.surface->vertex_buffer(), GL_WRITE_ONLY
            ));
            if (!vertex_buffer)
            {
                return false;
            }

            // Transform the released payload's mesh
            const auto& released_payload_p__asteroid = released_payload.get_position();
            const easy3d::Mat4<float> T__universe__released_payload = easy3d::Mat4<float>::rotation(q__universe__asteroid)
                * easy3d::Mat4<float>::translation(released_payload_p__asteroid)
            ;
            for (std::size_t i = 0; i < released_payload_mesh.triangle_points.size(); i++)
            {
                const auto p = get_dimensioned(
                    T__universe__released_payload * released_payload_mesh.triangle_points[i],
                    DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
                );
                vertex_buffer[i].x = p.x;
                vertex_buffer[i].y = p.y;
                vertex_buffer[i].z = p.z;
            }

            released_payload_mesh.surface->set_uniform_coloring(easy3d::vec4(0, 1, 0, 1));

            easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, released_payload_mesh.surface->vertex_buffer());
        }
        else
        {
            easy3d::vec3* const vertex_buffer = reinterpret_cast<easy3d::vec3*>(easy3d::VertexArrayObject::map_buffer(
                GL_ARRAY_BUFFER, released_payload_mesh.surface->vertex_buffer(), GL_WRITE_ONLY
            ));
            if (!vertex_buffer)
            {
                return false;
            }

            // Put all of the markers inside of the asteroid to effectively
            // hide them
            for (std::size_t j = 0; j < released_payload_mesh.triangle_points.size(); j++)
            {
                vertex_buffer[j].x = 0;
                vertex_buffer[j].y = 0;
                vertex_buffer[j].z = 0;
            }

            released_payload_mesh.surface->set_uniform_coloring(easy3d::vec4(0, 0, 0, 1));

            easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, released_payload_mesh.surface->vertex_buffer());
        }

        window.update();

        if (window.corotating_camera_with_asteroid())
        {
            window.camera()->setPosition(easy3d::vec3(0, 0, dimensions_scaler.get_dimensioned(
                3 * model.get_asteroid().alpha,
                DimensionsScaler::ScaleOpChain() * DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::DISTANCE)
            )));
            window.camera()->setViewDirection(easy3d::vec3(0, 0, -1));
            window.camera()->setOrientation(easy3d::Quat<float>(easy3d::Vec<3, float>(0, 0, 1), asteroid.get_rotation()));
        }

        return true;
    }

    AsteroidMiningViewer window;

    const DimensionsScaler& dimensions_scaler;
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
