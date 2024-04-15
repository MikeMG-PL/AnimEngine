#include "Ellipse.h"

#include "Globals.h"

#include <glm/trigonometric.hpp>
#include <glm/ext/scalar_constants.hpp>

#include "MeshFactory.h"

std::shared_ptr<Ellipse> Ellipse::create()
{
    auto ellipse = std::make_shared<Ellipse>(AK::Badge<Ellipse> {});

    return ellipse;
}

std::shared_ptr<Ellipse> Ellipse::create(float const center_x, float const center_z, float const radius_x,
                                         float const radius_z, i32 const segment_count, std::shared_ptr<Material> const &material)
{
    auto ellipse = std::make_shared<Ellipse>(AK::Badge<Ellipse> {}, center_x, center_z, radius_x, radius_z, segment_count, material);

    return ellipse;
}

Ellipse::Ellipse(AK::Badge<Ellipse>) : Model(default_material)
{
    m_draw_type = DrawType::LineLoop;
    m_meshes.emplace_back(create_ellipse());
}

Ellipse::Ellipse(AK::Badge<Ellipse>, float center_x, float center_z, float radius_x, float radius_z, i32 segment_count, std::shared_ptr<Material> const& material)
    : Model(material), m_center_x(center_x), m_center_z(center_z), m_radius_x(radius_x), m_radius_z(radius_z), m_segment_count(segment_count)
{
    m_draw_type = DrawType::LineLoop;
    m_meshes.emplace_back(create_ellipse());
}

std::string Ellipse::get_name() const
{
    std::string const name = typeid(decltype(*this)).name();
    return name.substr(6);
}

std::shared_ptr<Mesh> Ellipse::create_ellipse() const
{ 
    float const theta = 2 * glm::pi<float>() / static_cast<float>(m_segment_count); 
    float const c = glm::cos(theta);
    float const s = glm::sin(theta);

    float x = 1; // Start at angle = 0 
    float z = 0;

    std::vector<Vertex> vertices;

    for (i32 i = 0; i < m_segment_count; ++i) 
    {
        Vertex vertex = {};
        // Apply radius and offset
        vertex.position = glm::vec3(x * m_radius_x + m_center_x, 0.0f, z * m_radius_z + m_center_z);

        // Apply the rotation matrix
        float const t = x;
        x = c * x - s * z;
        z = s * t + c * z;

        vertices.emplace_back(vertex);
    }

    return MeshFactory::create(vertices, {}, {}, m_draw_type, m_material, DrawFunctionType::NotIndexed);
}
