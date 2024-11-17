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
    sample_database.clear();
    auto const assets = Editor::Editor::get_instance()->get_assets(Editor::AssetType::Animation);
    std::string const first_anim_path = assets->at(0).path;

    auto const temp_skinned_model =
        entity->add_component_internal<SkinnedModel>(SkinnedModel::create(skinned_model_path, first_anim_path, default_material));

    // Disable any animation system elements that might interfere with root bone data
    temp_skinned_model->enable_root_motion = false;
    AnimationEngine::get_instance()->allow_animation_previews = false;

    /// FORMULAS ////
    // sample_start_time = min_margin + features_num * sample_rate_ms
    // sample_end_time = max_margin - features_num * sample_rate_ms
    // num_samples = int(1 + (sample_end_time - sample_start_time) / sample_rate_ms)

    for (u32 asset_id = 0; asset_id < assets->size(); asset_id++)
    {
        log("Clip \"" + assets->at(asset_id).path + "\":");
        temp_skinned_model->anim_path = assets->at(asset_id).path;
        temp_skinned_model->reprepare();

        float const min_margin = 500.0f; // To prevent sampling T-pose from my dataset
        float const max_margin = temp_skinned_model->animation.duration;
        float const sample_rate_ms = sample_rate * 1000.0f;
        float const sample_start_time = min_margin + feature_num * sample_rate_ms;
        float const sample_end_time = max_margin - feature_num * sample_rate_ms;
        u32 const num_samples =
            (static_cast<u32>(sample_end_time) - static_cast<u32>(sample_start_time)) / static_cast<u32>(sample_rate_ms); // Double check this formula!

        if (sample_end_time - sample_start_time < 0)
        {
            log("Clip \"" + assets->at(asset_id).path + "\" too short to sample with current rate, skipping...",
                DebugType::Error);
            continue;
        }

        // Generating samples
        for (u32 i = 0; i < num_samples; i++)
        {
            log("   Sample " + std::to_string(i) + ":");
            // Going through individual features

            Sample sample = {};

            for (i32 j = -feature_num; j <= feature_num; j++)
            {
                float const step = static_cast<float>(j) * sample_rate_ms;

                temp_skinned_model->animation.current_time = sample_start_time + static_cast<float>(i) * sample_rate_ms + step;
                temp_skinned_model->calculate_bone_transform(&temp_skinned_model->animation.root_node, glm::mat4(1.0f));

                glm::vec3 feature_pos = calculate_feature_position(temp_skinned_model);
                glm::vec3 facing_direction = calculate_facing_direction(temp_skinned_model);

                Feature feature = {};
                feature.position = feature_pos;
                feature.facing_direction = facing_direction;

                if (j < 0)
                    sample.past_features.emplace_back(feature);

                if (j == 0)
                    sample.current_feature = feature;

                if (j > 0)
                    sample.future_features.emplace_back(feature);

                log("      Feature " + std::to_string(j) + ":" + " time: " + std::to_string(temp_skinned_model->animation.current_time)
                    + " ms, pos: " + std::to_string(feature_pos.x) + ", " + std::to_string(feature_pos.y) + ", "
                    + std::to_string(feature_pos.z) + ", facing direction: " + std::to_string(facing_direction.x) + ", "
                    + std::to_string(facing_direction.y) + ", " + std::to_string(facing_direction.z));
            }

            sample.clip_local_time = temp_skinned_model->animation.current_time;
            sample.clip_id = asset_id;

            sample_database.emplace_back(sample);
        }
    }

    /////////////////////////////////////////////////////////////////
    AnimationEngine::get_instance()->allow_animation_previews = true;
}

void MotionMatching::initialize()
{
    Component::initialize();
    log("Deserialized motion matching data with " + std::to_string(sample_database.size()) + " samples.");
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
