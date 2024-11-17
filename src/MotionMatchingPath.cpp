#include "MotionMatchingPath.h"

std::shared_ptr<MotionMatchingPath> MotionMatchingPath::create()
{
    return std::make_shared<MotionMatchingPath>(AK::Badge<MotionMatchingPath> {});
}

MotionMatchingPath::MotionMatchingPath(AK::Badge<MotionMatchingPath>)
{
}

void MotionMatchingPath::draw_editor()
{
    Curve::draw_editor();
}
