#include "AnimationEngine.h"

#include "AK/AK.h"
#include "Globals.h"
#include "Rig.h"

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

    for (auto const& skinned_model : m_skinned_models)
    {
        m_current_time += skinned_model->animation.ticks_per_second * delta_time; // you can apply play_rate here
        m_current_time = fmod(m_current_time, skinned_model->animation.duration);

        skinned_model->calculate_bone_transform(&skinned_model->animation.root_node, glm::mat4(1.0f));
        if (!skinned_model->skinning_matrices.empty())
        {
            // u16 const rotation_bone_id = 35;
            // auto const value = static_cast<float>(delta_time);
            // glm::mat4 rotation_matrix = glm::rotate(glm::mat4(1.0f), value, glm::vec3(0.0f, 1.0f, 0.0f));
            // skinned_model->skinning_matrices[rotation_bone_id] = rotation_matrix * skinned_model->skinning_matrices[rotation_bone_id];
            renderer_dx11->set_skinning_buffer(skinned_model, skinned_model->get_skinning_matrices());
        }
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

double AnimationEngine::get_current_time() const
{
    return m_current_time;
}
