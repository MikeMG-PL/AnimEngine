#pragma once
#include "MotionMatchingSampler.h"
#include "RendererDX11.h"
#include "SkinnedModel.h"

#include <memory>
#include <vector>

class AnimationEngine
{
public:
    friend Editor::Editor; // In order to have a global and centralized control over motion matching from the Motion Matching Custom Editor.
    AnimationEngine(AnimationEngine const&) = delete;
    void operator=(AnimationEngine const&) = delete;

    void initialize();
    void update_animations();
    void draw_animation_preview();
    void register_skinned_model(std::shared_ptr<SkinnedModel> const& skinned_model);
    void unregister_skinned_model(std::shared_ptr<SkinnedModel> const& skinned_model);
    void register_motion_matching_handler(std::shared_ptr<MotionMatchingSampler>& handler);
    void count_motion_matching_handlers(i8 delta);
    std::weak_ptr<MotionMatchingSampler> get_motion_matching_sampler() const;

    static std::shared_ptr<AnimationEngine> get_instance()
    {
        return m_instance;
    }

    [[nodiscard]] glm::mat4* get_skinning_matrices();

    AnimationEngine() = default;
    virtual ~AnimationEngine() = default;

    static void set_instance(std::shared_ptr<AnimationEngine> const& animation_engine)
    {
        m_instance = animation_engine;
    }

    bool allow_animation_previews = true;

private:
    inline static std::shared_ptr<AnimationEngine> m_instance;
    SkinningBuffer m_skinning_buffer = {};
    std::vector<std::shared_ptr<SkinnedModel>> m_skinned_models = {};
    std::weak_ptr<MotionMatchingSampler> m_motion_matching_sampler = {};
    u8 m_handler_count = 0;
};
