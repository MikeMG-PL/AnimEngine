#pragma once

#include <array>

#include <glm/vec2.hpp>

#include "Types.h"
#include "assimp/matrix4x4.h"
#include "glm/fwd.hpp"
#include "glm/gtc/quaternion.hpp"

namespace AK
{

struct xform
{
    glm::vec3 pos = {0.0f, 0.0f, 0.0f};
    glm::quat rot = {1.0f, 0.0f, 0.0f, 0.0f};
};

class Math
{
public:
    static glm::vec2 get_perpendicular_axis(std::array<glm::vec2, 4> const& passed_corners, u8 const index);
    static glm::vec2 get_normal(glm::vec2 const& v);
    static glm::vec2 project_on_axis(std::array<glm::vec2, 4> const& vertices, glm::vec2 const& axis);

    static float get_ranges_overlap_length(glm::vec2 const& a, glm::vec2 const& b);
    static bool are_ranges_overlapping(glm::vec2 const& a, glm::vec2 const& b);

    static bool are_nearly_equal(float const x, float const y, float const epsilon = 0.001f);

    static float ease_in_out_elastic(float const x);
    static float ease_out_quart(float const x);

    static xform mul_xforms(xform const& a, xform const& b);
    static xform mat4_to_xform(glm::mat4 const& m);
    static glm::mat4 xform_to_mat4(xform const& m);
    static glm::mat4 ai_matrix_to_glm(aiMatrix4x4 const& from);

    // Applies only for axis-aligned rectangles. Used mostly for buttons. Collider2D handles OBB in different way.
    static bool is_point_inside_rectangle(glm::vec2 const& point, std::array<glm::vec2, 4> const& rectangle_corners);

    // Maps range A to B
    static float map_range_clamped(float min_a, float max_a, float min_b, float max_b, float value);

    static glm::vec2 line_intersection(glm::vec2 const& point1, glm::vec2 const& point2, glm::vec2 const& point3, glm::vec2 const& point4);
};

}
