#pragma once
#include "Curve.h"
#include "DebugDrawing.h"
#include "MotionMatchingSampler.h"

#include <deque>
#include <stack>

constexpr u8 mm_controller_debug_entity_pool = 255;

class MotionMatchingController : public Component
{
public:
    static std::shared_ptr<MotionMatchingController> create();
    explicit MotionMatchingController(AK::Badge<MotionMatchingController>);

    virtual void initialize() override;
    virtual void update_editor() override;
    virtual void awake() override;
    virtual void update() override;

#if EDITOR
    virtual void draw_editor() override;
#endif

    void draw_path();
    void sample_in_runtime();
    void generate_first_queue();
    glm::vec3 editor_to_world_curve_pos(glm::vec2 const& editor_pos);
    glm::vec2 world_to_editor_curve_pos(glm::vec3 const& world_pos);
    glm::vec2 get_point_on_curve_by_index(u32 index);
    u32 get_nearest_point_on_curve_id(glm::vec3 const& position);
    glm::vec3 get_nearest_point_on_curve_pos(glm::vec3 const& position);

    std::weak_ptr<Entity> path_point_container = {};
    float path_scale = 1.0f;

private:
    std::vector<std::shared_ptr<Entity>> m_cached_line = {};
    std::stack<std::shared_ptr<Entity>> m_additional_debug_entity_pool = {};
    std::shared_ptr<Curve> m_motion_matching_path = nullptr;
    std::shared_ptr<std::vector<Sample>> m_sample_database_ref = nullptr;
    std::weak_ptr<SkinnedModel> m_skinned_model_ref = {};
    u16 m_line_points_num = 256;
    bool m_first_update_pass = true;

    float m_offline_average_root_step = 0.0f;

    // Online sampling
    std::deque<Feature> m_past_feature_register = {};
    float m_online_sample_rate = 0.0f; // Assigned from MotionMatchingSampler
    Sample m_current_online_sample = {};
    float m_time = 0.0f;
};
