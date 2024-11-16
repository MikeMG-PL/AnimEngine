#include "AnimationEngine.h"

#include "AK/AK.h"
#include "Editor.h"
#include "Entity.h"
#include "Globals.h"
#include "Rig.h"
#include "SceneSerializer.h"

void AnimationEngine::initialize()
{
    auto const animation_engine = std::make_shared<AnimationEngine>();
    set_instance(animation_engine);
}

void AnimationEngine::update_animations()
{
    auto const renderer_dx11 = RendererDX11::get_instance_dx11()->get_instance_dx11();

    if (renderer_dx11 == nullptr)
        return;

    for (u16 i = 0; i < m_skinned_models.size(); i++)
    {
        auto const skinned_model = m_skinned_models[i];
        skinned_model->animation.current_time +=
            skinned_model->animation.ticks_per_second * delta_time * 1.0f; // you can apply play_rate here

        skinned_model->calculate_bone_transform(&skinned_model->animation.root_node, glm::mat4(1.0f));
    }
}

void AnimationEngine::draw_animation_preview()
{
    auto const renderer_dx11 = RendererDX11::get_instance_dx11()->get_instance_dx11();

    if (renderer_dx11 == nullptr)
        return;

    for (u16 i = 0; i < m_skinned_models.size(); i++)
    {
        // There's no time updating in preview. You need to press play.
        auto const skinned_model = m_skinned_models[i];

        // This is for convenience, my dataset animations have T-pose in 0 frame, so this makes the preview draw one of the early frames.
        if (allow_animation_previews && AK::Math::are_nearly_equal(skinned_model->animation.current_time, 0.0f)
            && skinned_model->animation.duration > 500.0f)
        {
            skinned_model->animation.current_time = 500.0f;
        }

        if (allow_animation_previews)
            skinned_model->calculate_bone_transform(&skinned_model->animation.root_node, glm::mat4(1.0f));
    }
}

void AnimationEngine::register_skinned_model(std::shared_ptr<SkinnedModel> const& skinned_model)
{
    m_skinned_models.emplace_back(skinned_model);
}

void AnimationEngine::unregister_skinned_model(std::shared_ptr<SkinnedModel> const& skinned_model)
{
    AK::swap_and_erase(m_skinned_models, skinned_model);
}

void AnimationEngine::register_motion_matching_handler(std::shared_ptr<MotionMatching>& handler)
{
    m_motion_matching_settings = handler;
}

void AnimationEngine::count_motion_matching_handlers(i8 delta)
{
    m_handler_count += delta;
}

glm::mat4* AnimationEngine::get_skinning_matrices()
{
    return m_skinning_buffer.bones;
}
