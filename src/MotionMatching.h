#pragma once
#include "AK/Badge.h"
#include "AK/Types.h"
#include "Component.h"
#include "glm/vec3.hpp"

#include <memory>

// NOTE (IMPORTANT!) Motion Matching settings are saved in a prefab MotionMatching.txt
// This prefab needs to be on scene and its settings will be exposed to a global editor.

class SkinnedModel;
constexpr u8 feature_num = 3;

struct Feature
{
    glm::vec3 position = {};
    glm::vec3 facing_direction = {};
};

struct Sample
{
    std::vector<Feature> past_features = {};
    std::vector<Feature> future_features = {};
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

    void populate_sample_database();

    // Debug
    void log(std::string const& message, DebugType type = DebugType::Log);
    void clear_log();

    std::string skinned_model_path = "./res/models/enemy/enemy.gltf";
    float sample_rate = 0.2f; // In seconds

    // Debug
    NON_SERIALIZED
    std::vector<DebugMessage> debug_messages = {};
    bool always_latest_logs = true;

private:
    glm::vec3 calculate_feature_position(std::shared_ptr<SkinnedModel> const& model) const;
    glm::vec3 calculate_facing_direction(std::shared_ptr<SkinnedModel> const& model) const;
    std::vector<Sample> m_sample_database = {};
};
