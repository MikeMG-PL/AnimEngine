#pragma once
#include "RendererDX11.h"
#include "Rig.h"
#include "SkinnedModel.h"

#include <memory>
#include <vector>

class AnimationEngine
{
public:
    AnimationEngine(AnimationEngine const&) = delete;
    void operator=(AnimationEngine const&) = delete;

    void initialize();
    void update_animations();
    void calculate_bone_transform(AssimpNodeData const* node, glm::mat4 const& parent_transform);
    void register_skinned_model(std::shared_ptr<SkinnedModel> const& skinned_model);
    void unregister_skinned_model(std::shared_ptr<SkinnedModel> const& skinned_model);

    // For debugging:
    float time = 0.0f;

    static std::shared_ptr<AnimationEngine> get_instance()
    {
        return m_instance;
    }

    AnimationEngine() = default;
    virtual ~AnimationEngine() = default;

    static void set_instance(std::shared_ptr<AnimationEngine> const& animation_engine)
    {
        m_instance = animation_engine;
    }

private:
    inline static std::shared_ptr<AnimationEngine> m_instance;
    std::vector<std::shared_ptr<SkinnedModel>> m_skinned_models = {};
    Animation m_current_animation = {};
    std::vector<glm::mat4> m_final_bone_matrices = {};
};
