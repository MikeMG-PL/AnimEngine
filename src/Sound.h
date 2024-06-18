#pragma once

#include <glm/vec3.hpp>
#include <miniaudio.h>

#include "AK/Badge.h"
#include "AK/Types.h"
#include "Component.h"

class Sound final : public Component
{
public:
    static std::shared_ptr<Sound> create();
    static std::shared_ptr<Sound> create(std::string const& path);
    static std::shared_ptr<Sound> create(std::string const& path, glm::vec3 const direction, float const rolloff = 0.5f,
                                         ma_attenuation_model const attenuation = ma_attenuation_model_inverse);
    static std::shared_ptr<Sound> play_sound(std::string const& path);
    static std::shared_ptr<Sound> play_sound_at_location(std::string const& path, glm::vec3 const position, glm::vec3 direction,
                                                         float rolloff = 0.5f,
                                                         ma_attenuation_model attenuation = ma_attenuation_model_inverse);

    explicit Sound(AK::Badge<Sound>)
    {
    }

    void play();
    void stop();
    void stop_with_fade(u64 const milliseconds);
    virtual void update() override;

    bool is_positional = false;

private:
    ma_sound m_internal_sound = {};
};
