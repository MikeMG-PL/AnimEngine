#pragma once
#include "RendererDX11.h"
#include "SkinnedModel.h"

#include <memory>
#include <vector>

class AnimationEngine
{
public:
    AnimationEngine(AnimationEngine const&) = delete;
    void operator=(AnimationEngine const&) = delete;

    void initialize();
    void update_animations() const;
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
};
