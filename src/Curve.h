#pragma once

#include <glm/glm.hpp>

#include "AK/Badge.h"
#include "Component.h"

enum class PointsConnection
{
    Linear,
    CatmullRom
};

class Curve : public Component
{
public:
    static std::shared_ptr<Curve> create();

    explicit Curve(AK::Badge<Curve>);

#if EDITOR
    virtual void draw_editor() override;
#endif

    std::vector<glm::vec2> points = {}; // These are always the points created by user
    std::vector<glm::vec2> curve = {}; // This will be equal "points" vector IF LINEAR or will contain all step points creating the curve

    glm::vec2 get_point_at(float x) const;
    float get_y_at(float x) const;
    void add_points(std::initializer_list<glm::vec2> new_points);

    bool clamp_x = false; // Clamp to keep original behavior implemented by Milosz
    PointsConnection connection = PointsConnection::Linear;

protected:
    explicit Curve();

private:
    float length() const;
};
