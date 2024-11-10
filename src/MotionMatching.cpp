#include "MotionMatching.h"

#include "AnimationEngine.h"
#include "Editor.h"
#include "Entity.h"

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
    auto const assets = Editor::Editor::get_instance()->get_assets(Editor::AssetType::Animation);
    std::string const first_anim_path = assets->at(0).path;

    log("Obtained animation assets:");

    for (auto const& i : *assets)
    {
        log(i.path);
    }

    //entity->add_component_internal<SkinnedModel>(SkinnedModel::create(skinned_model_path, first_anim_path, default_material));
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
