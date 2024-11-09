#include "MotionMatching.h"

#include "AnimationEngine.h"

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
