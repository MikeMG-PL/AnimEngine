#pragma once
#include "AK/Badge.h"
#include "AK/Types.h"
#include "Component.h"
#include "glm/vec3.hpp"

#include <memory>

// NOTE (IMPORTANT!) Motion Matching settings are saved in a prefab MotionMatchingSampler.txt
// This prefab needs to be on scene and its settings will be exposed to a global editor.

class SkinnedModel;
constexpr u8 feature_num = 3;

struct Feature
{
    glm::vec3 root_position = {};
    glm::vec3 left_foot_position = {};
    glm::vec3 right_foot_position = {};
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

class MotionMatchingSampler : public Component
{
public:
    static std::shared_ptr<MotionMatchingSampler> create();
    explicit MotionMatchingSampler(AK::Badge<MotionMatchingSampler>);
    virtual ~MotionMatchingSampler() override;

    void populate_sample_database();
    virtual void initialize() override;

    // Debug
    void log(std::string const& message, DebugType type = DebugType::Log);
    void clear_log();

    std::string skinned_model_path = "./res/models/enemy/enemy.gltf";
    float sample_rate = 0.2f; // In seconds

    // Debug
    NON_SERIALIZED
    std::vector<DebugMessage> debug_messages = {};
    bool always_latest_logs = true;

    // The most important thing there indeed ;)
    std::vector<Sample> sample_database = {};
    float offline_average_root_step = 0.0f;
    float offline_accumulated_root_step = 0.0f;

private:
    std::vector<Feature> relativize_sample(Sample& sample) const;
    glm::vec3 calculate_feature_position(std::shared_ptr<SkinnedModel> const& model) const;
    std::vector<glm::vec3> calculate_feet_positions(std::shared_ptr<SkinnedModel> const& model) const;
    glm::vec3 calculate_facing_direction(std::shared_ptr<SkinnedModel> const& model) const;
    void calculate_average_root_step();
    void accumulate_root_step(u32 sample_id);
};
