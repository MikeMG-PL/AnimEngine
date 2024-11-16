#include "MotionMatching.h"

#include "AnimationEngine.h"
#include "Editor.h"
#include "Entity.h"
#include "glm/gtx/matrix_decompose.hpp"

std::shared_ptr<MotionMatching> MotionMatching::create()
{
    auto motion_matching_handler = std::make_shared<MotionMatching>(AK::Badge<MotionMatching> {});
    AnimationEngine::get_instance()->register_motion_matching_handler(motion_matching_handler);
    return motion_matching_handler;
}

MotionMatching::MotionMatching(AK::Badge<MotionMatching>)
{
    AnimationEngine::get_instance()->count_motion_matching_handlers(1);
}

MotionMatching::~MotionMatching()
{
    AnimationEngine::get_instance()->count_motion_matching_handlers(-1);
}

void MotionMatching::populate_sample_database()
{
    auto const assets = Editor::Editor::get_instance()->get_assets(Editor::AssetType::Animation);
    std::string const first_anim_path = assets->at(0).path;

    log("Processed animation assets:");

    for (auto const& i : *assets)
    {
        log(i.path);
    }

    auto const temp_skinned_model =
        entity->add_component_internal<SkinnedModel>(SkinnedModel::create(skinned_model_path, first_anim_path, default_material));

    // Disable any animation system elements that might interfere with root bone data
    temp_skinned_model->enable_root_motion = false;
    AnimationEngine::get_instance()->allow_animation_previews = false;

    /// FORMULAS ////
    // sample_start_time = min_margin + features_num * sample_rate_ms
    // sample_end_time = max_margin - features_num * sample_rate_ms
    // num_samples = int(1 + (sample_end_time - sample_start_time) / sample_rate_ms)

    float const min_margin = 500.0f; // To prevent sampling T-pose from my dataset
    float const max_margin = temp_skinned_model->animation.duration;
    float const sample_rate_ms = sample_rate * 1000.0f;
    float const sample_start_time = min_margin + feature_num * sample_rate_ms;
    float const sample_end_time = max_margin - feature_num * sample_rate_ms;
    u32 const num_samples = (static_cast<u32>(sample_end_time) - static_cast<u32>(sample_start_time)) / static_cast<u32>(sample_rate_ms);

    temp_skinned_model->animation.current_time = 600.0f;
    temp_skinned_model->calculate_bone_transform(&temp_skinned_model->animation.root_node, glm::mat4(1.0f));

    glm::vec3 feature_pos = calculate_feature_position(temp_skinned_model);
    glm::vec3 facing_direction = calculate_facing_direction(temp_skinned_model);

    /////////////////////////////////////////////////////////////////
    AnimationEngine::get_instance()->allow_animation_previews = true;
}

void MotionMatching::log(std::string const& message, DebugType type)
{
    std::string prefix;

    switch (type)
    {
    case DebugType::Log:
        prefix += "Log: ";
        break;
    case DebugType::Warning:
        prefix += "Warning: ";
        break;
    case DebugType::Error:
        prefix += "Error: ";
        break;
    default:
        std::unreachable();
    }

    debug_messages.emplace_back(type, prefix + message);
}

void MotionMatching::clear_log()
{
    debug_messages.clear();
}

glm::vec3 MotionMatching::calculate_feature_position(std::shared_ptr<SkinnedModel> const& model) const
{
    glm::mat4 const root_matrix = model->animation.bones[0].local_transform;
    glm::quat q = {};
    glm::vec3 pos = {};
    glm::vec3 scale = {};
    glm::vec3 skew = {};
    glm::vec4 perspective = {};

    decompose(root_matrix, scale, q, pos, skew, perspective);

    return pos;
}

glm::vec3 MotionMatching::calculate_facing_direction(std::shared_ptr<SkinnedModel> const& model) const
{
    glm::mat4 const hips_matrix = model->animation.bones[1].local_transform;
    glm::quat q = {};
    glm::vec3 pos = {};
    glm::vec3 scale = {};
    glm::vec3 skew = {};
    glm::vec4 perspective = {};

    decompose(hips_matrix, scale, q, pos, skew, perspective);

    auto facing_direction = q * glm::vec3(0.0f, 0.0f, -1.0f); // Z vector is forward, but negative for some reason (trial & error)
    facing_direction.y = 0.0f;
    facing_direction = normalize(facing_direction);

    return facing_direction;
}
