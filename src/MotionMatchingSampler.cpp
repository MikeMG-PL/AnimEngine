#include "MotionMatchingSampler.h"

#include "AnimationEngine.h"
#include "Editor.h"
#include "Entity.h"
#include "glm/gtx/matrix_decompose.hpp"

std::shared_ptr<MotionMatchingSampler> MotionMatchingSampler::create()
{
    auto motion_matching_handler = std::make_shared<MotionMatchingSampler>(AK::Badge<MotionMatchingSampler> {});
    AnimationEngine::get_instance()->register_motion_matching_handler(motion_matching_handler);
    return motion_matching_handler;
}

MotionMatchingSampler::MotionMatchingSampler(AK::Badge<MotionMatchingSampler>)
{
    AnimationEngine::get_instance()->count_motion_matching_handlers(1);
}

MotionMatchingSampler::~MotionMatchingSampler()
{
    AnimationEngine::get_instance()->count_motion_matching_handlers(-1);
}

void MotionMatchingSampler::populate_sample_database()
{
    sample_database.clear();
    auto const assets = Editor::Editor::get_instance()->get_assets(Editor::AssetType::Animation);
    std::string const first_anim_path = assets->at(0).path;

    std::shared_ptr<SkinnedModel> temp_skinned_model = nullptr;

    if (entity->get_component<SkinnedModel>() == nullptr)
    {
        temp_skinned_model =
            entity->add_component_internal<SkinnedModel>(SkinnedModel::create(skinned_model_path, first_anim_path, default_material));
    }
    else
    {
        temp_skinned_model = entity->get_component<SkinnedModel>();
    }

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
        u32 const num_samples = (static_cast<u32>(sample_end_time) - static_cast<u32>(sample_start_time))
                              / static_cast<u32>(sample_rate_ms); // Double check this formula!

        if (sample_end_time - sample_start_time < 0)
        {
            log("Clip \"" + assets->at(asset_id).path + "\" too short to sample with current rate, skipping...", DebugType::Error);
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
            }

            sample.clip_local_time = temp_skinned_model->animation.current_time;
            sample.clip_id = asset_id;

            // Sample relativized, returning feature vector for convenient display
            std::vector<Feature> features = relativize_sample(sample);

            for (i32 j = -feature_num; j <= feature_num; j++)
            {
                Feature const f = features[j + feature_num];

                log("      Feature " + std::to_string(j) + ":"
                    + " time: " + std::to_string(temp_skinned_model->animation.current_time + j * sample_rate_ms)
                    + " ms, pos: " + std::to_string(f.position.x) + ", " + std::to_string(f.position.y) + ", "
                    + std::to_string(f.position.z) + ", facing direction: " + std::to_string(f.facing_direction.x) + ", "
                    + std::to_string(f.facing_direction.y) + ", " + std::to_string(f.facing_direction.z));
            }

            sample_database.emplace_back(sample);
        }
    }

    /////////////////////////////////////////////////////////////////
    AnimationEngine::get_instance()->allow_animation_previews = true;
}

void MotionMatchingSampler::initialize()
{
    Component::initialize();
    log("Deserialized motion matching data with " + std::to_string(sample_database.size()) + " samples.");
}

void MotionMatchingSampler::log(std::string const& message, DebugType type)
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

void MotionMatchingSampler::clear_log()
{
    debug_messages.clear();
}

std::vector<Feature> MotionMatchingSampler::relativize_sample(Sample& sample) const
{
    std::vector<Feature> f = {};

    // Animation clip space
    glm::vec3 const current_pos_clip_space = sample.current_feature.position;
    glm::vec3 const current_facing_dir_clip_space = sample.current_feature.facing_direction;

    // Transforming to feature space
    sample.current_feature.position = {0.0f, 0.0f, 0.0f};
    sample.current_feature.facing_direction = {0.0f, 0.0f, 1.0f}; // Forward facing direction

    for (auto& past_feature : sample.past_features)
    {
        past_feature.position = current_pos_clip_space - past_feature.position;
        past_feature.facing_direction = AK::Math::transform_to_new_space_z(past_feature.facing_direction, current_facing_dir_clip_space);
        f.emplace_back(past_feature);
    }

    f.emplace_back(sample.current_feature);

    for (auto& future_feature : sample.future_features)
    {
        future_feature.position = current_pos_clip_space - future_feature.position;
        future_feature.facing_direction =
            AK::Math::transform_to_new_space_z(future_feature.facing_direction, current_facing_dir_clip_space);
        f.emplace_back(future_feature);
    }

    return f;
}

glm::vec3 MotionMatchingSampler::calculate_feature_position(std::shared_ptr<SkinnedModel> const& model) const
{
    // Choose not the 0-th matrix (turned out that the order might be a bit off...) but FIND ROOT MATRICES by name
    // Of course finding by name is bug-prone, but this looks as the best quick fix for now
    u32 root_id = 0;

    for (u32 i = 0; i < model->animation.bones.size(); i++)
    {
        if (model->animation.bones[i].name.contains("root") || model->animation.bones[i].name.contains("Root"))
        {
            root_id = i;
            break;
        }
    }

    glm::mat4 const root_matrix = model->animation.bones[root_id].local_transform;
    glm::quat q = {};
    glm::vec3 pos = {};
    glm::vec3 scale = {};
    glm::vec3 skew = {};
    glm::vec4 perspective = {};

    decompose(root_matrix, scale, q, pos, skew, perspective);

    return pos;
}

glm::vec3 MotionMatchingSampler::calculate_facing_direction(std::shared_ptr<SkinnedModel> const& model) const
{
    // Choose not the 1-th matrix (turned out that the order might be a bit off...) but FIND HIPS MATRICES by name
    // Of course finding by name is bug-prone, but this looks as the best quick fix for now
    u32 hips_id = 0;

    for (u32 i = 0; i < model->animation.bones.size(); i++)
    {
        if (model->animation.bones[i].name.contains("hips") || model->animation.bones[i].name.contains("Hips"))
        {
            hips_id = i;
            break;
        }
    }

    glm::mat4 const hips_matrix = model->animation.bones[hips_id].local_transform;
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
