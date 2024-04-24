#pragma once

#include "Component.h"
#include "Engine.h"
#include "Serialization.h"
#include "Input.h"
#include "Window.h"

class LighthouseLight final : public Component
{
public:
    static std::shared_ptr<LighthouseLight> create();

    explicit LighthouseLight(AK::Badge<LighthouseLight>);

    virtual void awake() override;
    virtual void update() override;
    virtual void draw_editor() override;

    glm::vec2 get_position() const;

    float range = 2.5f;
    float additional_ship_speed = 0.0003f;

private:
    float const m_playfield_width = 5.4f;
    float const m_playfield_additional_width = 1.4f;
    float const m_playfield_height = 3.6f;
};
