#pragma once
#include "AK/Badge.h"
#include "AK/Types.h"
#include "Component.h"
#include "glm/vec3.hpp"

#include <memory>

// NOTE (IMPORTANT!) Motion Matching settings are saved in a prefab MotionMatching.txt
// This prefab needs to be on scene and its settings will be exposed to a global editor.

constexpr u8 feature_count = 3;

struct Feature
{
    glm::vec3 position = {};
    glm::vec3 facing_direction = {};
};

struct Sample
{
    Feature past_features[feature_count] = {};
    Feature future_features[feature_count] = {};
    Feature current_feature = {};
    u32 clip_id = 0;
    float clip_local_time = 0.0f;
};

class MotionMatching : public Component
{
public:
    static std::shared_ptr<MotionMatching> create();
    explicit MotionMatching(AK::Badge<MotionMatching>);
    virtual ~MotionMatching() override;

    NON_SERIALIZED
    std::vector<Sample> m_sample_database = {};
    float sample_rate = 0.2f; // In seconds
};
