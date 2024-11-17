#include "Math.h"

#include "glm/fwd.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"

#include <corecrt_math_defines.h>
#include <glad/glad.h>
#include <glm/gtc/epsilon.hpp>

namespace AK
{

glm::vec2 Math::get_normal(glm::vec2 const& v)
{
    glm::vec2 const norm = {-v.y, v.x};
    return glm::normalize(norm);
}

glm::vec2 Math::get_perpendicular_axis(std::array<glm::vec2, 4> const& passed_corners, u8 const index)
{
    u8 const first = index;
    u8 next = index + 1;

    if (index + 1 == 4)
        next = 0;

    return get_normal(passed_corners[next] - passed_corners[first]);
}

glm::vec2 Math::project_on_axis(std::array<glm::vec2, 4> const& vertices, glm::vec2 const& axis)
{
    float min = std::numeric_limits<float>::infinity();
    float max = -std::numeric_limits<float>::infinity();

    for (u8 i = 0; i < 4; ++i)
    {
        float const projection = glm::dot(vertices[i], axis);

        if (projection < min)
        {
            min = projection;
        }

        if (projection > max)
        {
            max = projection;
        }
    }

    return {min, max};
}

bool Math::are_ranges_overlapping(glm::vec2 const& a, glm::vec2 const& b)
{
    // A and B are ranges and it's assumed that a.x <= a.y and b.x <= b.y
    return a.x <= b.y && a.y >= b.x;
}

bool Math::is_point_inside_rectangle(glm::vec2 const& point, std::array<glm::vec2, 4> const& rectangle_corners)
{
    float const x = point.x;
    float const y = point.y;

    float const minX =
        std::min(rectangle_corners[0].x, std::min(rectangle_corners[1].x, std::min(rectangle_corners[2].x, rectangle_corners[3].x)));
    float const maxX =
        std::max(rectangle_corners[0].x, std::max(rectangle_corners[1].x, std::max(rectangle_corners[2].x, rectangle_corners[3].x)));
    float const minY =
        std::min(rectangle_corners[0].y, std::min(rectangle_corners[1].y, std::min(rectangle_corners[2].y, rectangle_corners[3].y)));
    float const maxY =
        std::max(rectangle_corners[0].y, std::max(rectangle_corners[1].y, std::max(rectangle_corners[2].y, rectangle_corners[3].y)));

    return x >= minX && x <= maxX && y >= minY && y <= maxY;
}

float Math::map_range_clamped(float const min_a, float const max_a, float const min_b, float const max_b, float value)
{
    // First, clamp the value within range A to handle values outside the range
    value = std::max(min_a, std::min(max_a, value));

    // Map the clamped value from range A to range B
    float const mapped_value = min_b + (value - min_a) * (max_b - min_b) / (max_a - min_a);

    // Clamp the mapped value within range B and return
    return std::max(min_b, std::min(max_b, mapped_value));
}

float Math::get_ranges_overlap_length(glm::vec2 const& a, glm::vec2 const& b)
{
    // A and B are ranges and it's assumed that a.x <= a.y and b.x <= b.y
    if (!are_ranges_overlapping(a, b))
    {
        return 0.f;
    }

    return std::min(a.y, b.y) - std::max(a.x, b.x);
}

bool Math::are_nearly_equal(float const x, float const y, float const epsilon)
{
    return glm::epsilonEqual(x, y, epsilon);
}

float Math::ease_out_quart(float const x)
{
    return 1.0f - pow(1.0f - x, 4.0f);
}

std::vector<glm::vec2> Math::catmull_rom_curve(std::vector<glm::vec2> const& points, u32 num_segments)
{
    std::vector<glm::vec2> curve;

    if (points.size() < 2)
        return curve;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
        glm::vec2 p0 = (i == 0) ? points[i] : points[i - 1];
        glm::vec2 p1 = points[i];
        glm::vec2 p2 = points[i + 1];
        glm::vec2 p3 = (i + 2 < points.size()) ? points[i + 2] : points[i + 1];

        for (int j = 0; j < num_segments; ++j)
        {
            float t = j / static_cast<float>(num_segments);

            // Catmull-Rom formula
            float const t2 = t * t;
            float const t3 = t2 * t;

            glm::vec2 point =
                0.5f
                * ((2.0f * p1) + (-p0 + p2) * t + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 + (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
            curve.push_back(point);
        }
    }

    // Add the last point
    curve.push_back(points.back());

    return curve;
}

xform Math::mul_xforms(xform const& a, xform const& b)
{
    xform result;
    result.pos = a.pos + b.pos;
    result.rot = a.rot * b.rot;
    return result;
}

xform Math::mat4_to_xform(glm::mat4 const& m)
{
    xform xf;

    // Extract translation
    xf.pos = glm::vec3(m[3]); // The translation vector is in the fourth column

    // Extract rotation
    glm::mat3 const rotationMatrix = glm::mat3(m); // Extract the upper 3x3 submatrix
    xf.rot = glm::quat_cast(rotationMatrix); // Convert 3x3 matrix to quaternion

    return xf;
}

glm::mat4 Math::ai_matrix_to_glm(aiMatrix4x4 const& from)
{
    auto to = glm::mat4(1.0f);
    to[0][0] = static_cast<GLfloat>(from.a1);
    to[0][1] = static_cast<GLfloat>(from.b1);
    to[0][2] = static_cast<GLfloat>(from.c1);
    to[0][3] = static_cast<GLfloat>(from.d1);
    to[1][0] = static_cast<GLfloat>(from.a2);
    to[1][1] = static_cast<GLfloat>(from.b2);
    to[1][2] = static_cast<GLfloat>(from.c2);
    to[1][3] = static_cast<GLfloat>(from.d2);
    to[2][0] = static_cast<GLfloat>(from.a3);
    to[2][1] = static_cast<GLfloat>(from.b3);
    to[2][2] = static_cast<GLfloat>(from.c3);
    to[2][3] = static_cast<GLfloat>(from.d3);
    to[3][0] = static_cast<GLfloat>(from.a4);
    to[3][1] = static_cast<GLfloat>(from.b4);
    to[3][2] = static_cast<GLfloat>(from.c4);
    to[3][3] = static_cast<GLfloat>(from.d4);
    return to;
}

glm::mat4 Math::xform_to_mat4(xform const& m)
{
    // Create a 4x4 identity matrix
    auto result = glm::mat4(1.0f);

    // Convert quaternion rotation to a 3x3 rotation matrix and place it in the upper-left corner
    glm::mat4 const rotationMatrix = glm::toMat4(m.rot);

    // Set the rotation part of the matrix
    result *= rotationMatrix;

    // Set the translation part of the matrix (last column)
    result[3] = glm::vec4(m.pos, 1.0f);

    return result;
}

float Math::ease_in_out_elastic(float const x)
{
    float constexpr c5 = (2.0f * M_PI) / 4.5f;

    return are_nearly_equal(x, 0.0f) ? 0.0f
         : are_nearly_equal(x, 1.0f) ? 1.0f
         : x < 0.5f                  ? -(pow(2.0f, 20.0f * x - 10.0f) * sin((20.0f * x - 11.125f) * c5)) / 2.0f
                                     : (pow(2.0f, -20.0f * x + 10.0f) * sin((20.0f * x - 11.125f) * c5)) / 2.0f + 1.0f;
}

glm::vec2 Math::line_intersection(glm::vec2 const& point1, glm::vec2 const& point2, glm::vec2 const& point3, glm::vec2 const& point4)
{
    float const x1 = point1.x, x2 = point2.x, x3 = point3.x, x4 = point4.x;
    float const y1 = point1.y, y2 = point2.y, y3 = point3.y, y4 = point4.y;

    // Calculate the determinant of the coefficient matrix.
    float const det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (glm::abs(det) < 0.001f) // epsilon
    {
        // The lines are parallel (or coincident, if the segments overlap).
        // Shouldn't be vector zero but it won't be used when there's no intersection so whatever.
        return {0.0f, 0.0f};
    }

    // Calculate the x and y coordinates of the intersection point.
    glm::vec2 intersection;
    intersection.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / det;
    intersection.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / det;

    return intersection;
}

}
