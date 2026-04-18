#pragma once

#include "dimensions_scaler.hpp"
#include "model.hpp"

#include <easy3d/algo/surface_mesh_factory.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/vertex_array_object.h>
#include <easy3d/viewer/viewer.h>

#include <chrono>
#include <memory>

namespace am {

class View
{
public:
    View(const DimensionsScaler& dimensions_scaler_, Model& model_) :
        window("Triaxial Ellipsoid Viewer"),
        dimensions_scaler(dimensions_scaler_),
        model(model_),
        asteroid_mesh(easy3d::SurfaceMeshFactory::icosphere()),
        asteroid_mesh_surface(std::make_shared<easy3d::TrianglesDrawable>("asteroid_mesh_faces")),
        start_animation_time(),
        last_animation_time()
    {
        window.resize(1600, 1200);

        const Asteroid& asteroid = model.get_asteroid();
        for (auto v : asteroid_mesh.vertices())
        {
            auto& p = asteroid_mesh.position(v);
            p.x *= asteroid.alpha;
            p.y *= asteroid.beta;
            p.z *= asteroid.gamma;
        }

        asteroid_mesh_triangle_points = asteroid_mesh.points();
        for (auto fh : asteroid_mesh.faces())
        {
            auto circulator = asteroid_mesh.vertices(fh);
            auto end = circulator;
            do {
                asteroid_mesh_triangle_indices.push_back((*circulator).idx());
                ++circulator;
            } while (circulator != end);
        }

        asteroid_mesh_surface->update_vertex_buffer(asteroid_mesh_triangle_points, true);
        asteroid_mesh_surface->update_element_buffer(asteroid_mesh_triangle_indices);
        window.add_drawable(asteroid_mesh_surface);
        window.animation_func_= [this](easy3d::Viewer*) -> bool { return this->animation_callback(); };
        window.set_animation(true);
    }

    int run()
    {
        start_animation_time = std::chrono::system_clock::now();
        last_animation_time = start_animation_time;
        return window.run();
    }

protected:
    bool animation_callback()
    {
        const auto now = std::chrono::system_clock::now();
        const double dt = std::chrono::duration<double>(now - last_animation_time).count();
        model.update(dt);
        const bool rc = render_model();
        last_animation_time = now;
        return rc;
    }

    bool render_model()
    {
        // map the vertex buffer into the client's address space
        void* pointer = easy3d::VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, asteroid_mesh_surface->vertex_buffer(), GL_WRITE_ONLY);

        easy3d::vec3* vertices = reinterpret_cast<easy3d::vec3*>(pointer);
        if (!vertices)
            return false;

        const double asteroid_rotation_rads = dimensions_scaler.get_dimensioned(
            model.get_asteroid().get_rotation(),
            DimensionsScaler::ScaleOpChain() / DimensionsScaler::ScaleFactor(DimensionsScaler::ScaleFactor::DimensionType::TIME)
        );
        easy3d::Quat<float> asteroid_rotation_q(easy3d::Vec<3, float>(0, 0, 1), asteroid_rotation_rads);
        for (std::size_t i = 0; i < asteroid_mesh_triangle_points.size(); i++)
        {
            const auto p = asteroid_rotation_q.rotate(asteroid_mesh_triangle_points[i]);
            vertices[i].x = p.x;
            vertices[i].y = p.y;
            vertices[i].z = p.z;
        }

        // unmap the vertex buffer
        easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, asteroid_mesh_surface->vertex_buffer());

        window.update();
        return true;
    }

    easy3d::Viewer window;

    const DimensionsScaler& dimensions_scaler;
    Model& model;

    easy3d::SurfaceMesh asteroid_mesh;
    std::vector<easy3d::vec3> asteroid_mesh_triangle_points;
    std::vector<unsigned int> asteroid_mesh_triangle_indices;
    std::shared_ptr<easy3d::TrianglesDrawable> asteroid_mesh_surface;

    std::chrono::time_point<std::chrono::system_clock> start_animation_time;
    std::chrono::time_point<std::chrono::system_clock> last_animation_time;
};

}
