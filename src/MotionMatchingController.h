#pragma once
#include "Curve.h"
#include "DebugDrawing.h"
#include "Editor.h"
#include "MotionMatchingSampler.h"

#include <chrono>
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

    glm::vec3 editor_to_world_curve_pos(glm::vec2 const& editor_pos);
    glm::vec2 world_to_editor_curve_pos(glm::vec3 const& world_pos);
    glm::vec2 get_point_on_curve_by_index(u32 index);
    u32 get_nearest_point_on_curve_id(glm::vec3 const& position);
    glm::vec3 get_nearest_point_on_curve_pos(glm::vec3 const& position);

    std::weak_ptr<Entity> path_point_container = {};
    float path_scale = 1.0f;

private:
    void draw_path();
    void sample_in_runtime();
    void generate_first_queue();
    void choose_best_sample(Sample const& online_sample, glm::vec3 const& realignment_vector);
    void measure_root_deviation();
    void measure_execution_time(float current_execution_time);

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
    float m_previous_cost = FLT_MAX; // For choosing the best sample
    std::shared_ptr<std::vector<Editor::Asset>> m_assets = nullptr;
    glm::vec3 m_cached_pos = glm::vec3(0.0f);

    i32 m_previous_best_sample_id = -1;
    u32 m_cost_lock_counter = 0;
    Sample m_best_sample = {};

    // Measurements
    u32 m_root_deviation_measures_num = 0;
    float m_accumulated_root_deviation = 0.0f;

    u32 m_execution_time_measures_num = 0;
    float m_accumulated_execution_time = 0.0f;
};
