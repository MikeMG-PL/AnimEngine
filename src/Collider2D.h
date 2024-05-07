#pragma once

#include "Component.h"
#include "AK/AK.h"
#include "AK/Types.h"
#include "AK/Badge.h"
#include "glm/glm.hpp"

#include <array>

enum class ColliderType2D
{
    Rectangle,
    Circle
};

class Collider2D final : public Component
{
public:
    static std::shared_ptr<Collider2D> create();
    static std::shared_ptr<Collider2D> create(float const radius, bool const is_static = false);
    static std::shared_ptr<Collider2D> create(glm::vec2 const bounds_dimensions, bool const is_static = false);
    static std::shared_ptr<Collider2D> create(float const width, float const height, bool const is_static = false);

    bool is_point_inside_obb(glm::vec2 const& point, std::array<glm::vec2, 4> const& rectangle_corners) const;

    // CircleCollision
    explicit Collider2D(AK::Badge<Collider2D>, float const radius, bool const is_static);

    // RectangleCollision
    explicit Collider2D(AK::Badge<Collider2D>, glm::vec2 const bounds_dimensions, bool const is_static);
    explicit Collider2D(AK::Badge<Collider2D>, float const width, float const height, bool const is_static);

    virtual void initialize() override;
    virtual void uninitialize() override;

    virtual void start() override;
    virtual void update() override;

    // CIRCLE X CIRCLE
    void separate(Collider2D const& other) const;

    // RECTANGLE X RECTANGLE, CIRCLE X RECTANGLE
    void separate(bool const sign) const;

    ColliderType2D get_collider_type() const;
    float get_radius_2d() const;
    glm::vec2 get_center_2d() const;
    glm::vec2 get_bounds_dimensions_2d() const;
    bool overlaps(Collider2D& other);

    bool is_static() const;
    void set_static(bool const value);

private:
    glm::vec2 get_perpendicular_axis(std::array<glm::vec2, 4> const& passed_corners, u8 const index) const;
    glm::vec2 get_normal(glm::vec2 const& v) const;
    glm::vec2 project_on_axis(std::array<glm::vec2, 4> const& vertices, glm::vec2 const& axis) const;
    glm::vec2 line_intersection(glm::vec2 const& point1, glm::vec2 const& point2, glm::vec2 const& point3, glm::vec2 const& point4) const;
    void compute_axes(glm::vec2 const& center, float const angle);
    bool intersect_circle(glm::vec2 const& center, float const radius, glm::vec2 const& p1, glm::vec2 const& p2);

    bool test_collision_rectangle_rectangle(Collider2D const& obb1, Collider2D const& obb2);
    bool test_collision_circle_circle(Collider2D const& obb1, Collider2D const& obb2) const;
    bool test_collision_circle_rectangle(Collider2D& obb1, Collider2D& obb2);

    ColliderType2D m_collider_type = ColliderType2D::Circle;
    bool m_is_static = false;

    std::array<glm::vec2, 4> m_corners = {}; // For rectangle, calculated each frame
    std::array<glm::vec2, 2> m_axes = {};    // For rectangle, calculated each frame

    float m_width = 1.0f;  // For rectangle
    float m_height = 1.0f; // For rectangle

    float m_radius = 0.0f; // For circle
    glm::vec2 m_mtv = {};  // Minimal translation vector
};
