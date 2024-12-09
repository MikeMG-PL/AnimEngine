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
    virtual void awake() override;

#if EDITOR
    virtual void draw_editor() override;
#endif

    void draw_path();
    void sample_in_runtime();
    Sample generate_first_sample();
    glm::vec3 editor_to_world_curve_pos(glm::vec2 const& editor_pos);
    glm::vec2 get_point_at_curve_by_index(u32 const index);

    std::weak_ptr<Entity> path_point_container = {};
    float path_scale = 1.0f;

private:
    std::vector<std::shared_ptr<Entity>> m_cached_line = {};
    std::shared_ptr<Curve> m_motion_matching_path = nullptr;
    std::shared_ptr<std::vector<Sample>> m_sample_database_ref = nullptr;
    u16 m_line_points_num = 256;
    bool m_first_update_pass = true;
    float m_online_sample_rate = 0.0f;
};
