#pragma once
#include "Curve.h"
#include "DebugDrawing.h"

struct Sample;

class MotionMatchingController : public Component
{
public:
    static std::shared_ptr<MotionMatchingController> create();
    explicit MotionMatchingController(AK::Badge<MotionMatchingController>);

    virtual void initialize() override;
    virtual void update_editor() override;

#if EDITOR
    virtual void draw_editor() override;
#endif

    void draw_path();

    float path_scale = 1.0f;

private:
    std::vector<std::shared_ptr<Entity>> m_cached_line = {};
    std::shared_ptr<Curve> m_motion_matching_path = nullptr;
    std::shared_ptr<std::vector<Sample>> m_sample_database_ref = nullptr;
    u16 m_line_points_num = 256;
    bool m_first_update_pass = true;
};
