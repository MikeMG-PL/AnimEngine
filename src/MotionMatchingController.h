#pragma once
#include "Curve.h"

struct Sample;

class MotionMatchingController : public Component
{
public:
    static std::shared_ptr<MotionMatchingController> create();
    explicit MotionMatchingController(AK::Badge<MotionMatchingController>);

    virtual void initialize() override;

#if EDITOR
    virtual void draw_editor() override;
#endif

    void draw_path();

    float path_scale = 1.0f;

private:
    std::shared_ptr<Curve> motion_matching_path = nullptr;
    std::shared_ptr<std::vector<Sample>> m_sample_database_ref = nullptr;
};
