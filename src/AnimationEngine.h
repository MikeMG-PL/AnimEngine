#pragma once
#include "MotionMatching.h"
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
    void draw_animation_preview() const;
    void register_skinned_model(std::shared_ptr<SkinnedModel> const& skinned_model);
    void unregister_skinned_model(std::shared_ptr<SkinnedModel> const& skinned_model);
    void register_motion_matching_handler(std::shared_ptr<MotionMatching>& handler);
    void count_motion_matching_handlers(i8 delta);

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
    std::weak_ptr<MotionMatching> m_motion_matching_settings = {};
    u8 m_handler_count = 0;
};
