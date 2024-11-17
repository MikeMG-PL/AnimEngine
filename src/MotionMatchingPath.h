#pragma once
#include "Curve.h"
class MotionMatchingPath final : public Curve
{
public:
    static std::shared_ptr<MotionMatchingPath> create();
    explicit MotionMatchingPath(AK::Badge<MotionMatchingPath>);

#if EDITOR
    virtual void draw_editor() override;
#endif
};
