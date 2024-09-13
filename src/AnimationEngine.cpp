#include "AnimationEngine.h"

#include "AK/AK.h"
#include "Globals.h"

void AnimationEngine::initialize()
{
    auto const animation_engine = std::make_shared<AnimationEngine>();
    set_instance(animation_engine);
}

void AnimationEngine::update_animations() const
{
    auto const renderer_dx11 = RendererDX11::get_instance_dx11()->get_instance_dx11();

    if (renderer_dx11 == nullptr)
        return;

    for (auto const& skinned_model : m_skinned_models)
    {
        if (skinned_model->get_skinning_matrices())
        {
            u16 const rotation_bone_id = 35;
            auto const value = static_cast<float>(delta_time);
            glm::mat4 rotation_matrix = glm::rotate(glm::mat4(1.0f), value, glm::vec3(0.0f, 1.0f, 0.0f));
            skinned_model->skinning_matrices[rotation_bone_id] = rotation_matrix * skinned_model->skinning_matrices[rotation_bone_id];
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
