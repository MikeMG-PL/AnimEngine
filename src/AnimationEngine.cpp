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
        calculate_bone_transform(&m_current_animation.root_node, glm::mat4(1.0f));
        if (!m_final_bone_matrices.empty())
        {
            // u16 const rotation_bone_id = 35;
            // auto const value = static_cast<float>(delta_time);
            // glm::mat4 rotation_matrix = glm::rotate(glm::mat4(1.0f), value, glm::vec3(0.0f, 1.0f, 0.0f));
            // skinned_model->skinning_matrices[rotation_bone_id] = rotation_matrix * skinned_model->skinning_matrices[rotation_bone_id];

            renderer_dx11->set_skinning_buffer(skinned_model, m_final_bone_matrices.data());
        }
    }
}

void AnimationEngine::calculate_bone_transform(AssimpNodeData const* node, glm::mat4 const& parent_transform)
{
    std::string const node_name = node->name;
    glm::mat4 node_transform = node->transformation;

    if (Bone* bone = m_current_animation.find_bone(node_name))
    {
        bone->update(0.0f);
        node_transform = bone->local_transform;
    }

    glm::mat4 const global_transformation = parent_transform * node_transform;

    auto bone_info_map = m_current_animation.bone_info_map;
    if (bone_info_map.contains(node_name))
    {
        i32 const index = bone_info_map[node_name].id;
        glm::mat4 const offset = bone_info_map[node_name].offset;
        m_final_bone_matrices[index] = global_transformation * offset;
    }

    for (int i = 0; i < node->children_count; i++)
        calculate_bone_transform(&node->children[i], global_transformation);
}

void AnimationEngine::register_skinned_model(std::shared_ptr<SkinnedModel> const& skinned_model)
{
    m_skinned_models.emplace_back(skinned_model);

    Animation animation = {};
    animation.init("./res/models/enemy/AS_Walking.gltf", skinned_model.get());

    m_current_animation = animation;

    for (u32 i = 0; i < 512; i++)
        m_final_bone_matrices.emplace_back(1.0f);
}

void AnimationEngine::unregister_skinned_model(std::shared_ptr<SkinnedModel> const& skinned_model)
{
    AK::swap_and_erase(m_skinned_models, skinned_model);
}
