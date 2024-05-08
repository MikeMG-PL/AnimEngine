#pragma once

#include "Component.h"

class Curve : public Component
{
public:
    static std::shared_ptr<Curve> create();

    explicit Curve(AK::Badge<Curve>);

    virtual void draw_editor() override;
    
    std::vector<glm::vec2> points = {};
    glm::vec2 get_point_at(float x) const;
    void add_points(std::initializer_list<glm::vec2> new_points);

protected:
    explicit Curve();

private:
    float length() const;
};