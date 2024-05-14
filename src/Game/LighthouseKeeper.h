#pragma once

#include "Component.h"
#include "Input.h"

class Port;
class Factory;
class Lighthouse;

class LighthouseKeeper final : public Component
{
public:
    static std::shared_ptr<LighthouseKeeper> create();

    explicit LighthouseKeeper(AK::Badge<LighthouseKeeper>);

    virtual void awake() override;
    virtual void update() override;
    virtual void draw_editor() override;

    bool is_inside_port() const;
    void set_is_inside_port(bool const value);

    float maximum_speed = 3.17f;
    float acceleration = 0.14f;
    float deceleration = acceleration;

    float interact_with_factory_distance = 0.5f;

    std::weak_ptr<Lighthouse> lighthouse = {};
    std::weak_ptr<Port> port = {};

private:
    void handle_input() const;

    bool m_is_inside_port = false;

    glm::vec2 m_speed = glm::vec2(0.0f, 0.0f);
};
