#include "MotionMatchingController.h"

#include "AK/AK.h"
#include "AnimationEngine.h"
#include "Entity.h"
#include <chrono>

std::shared_ptr<MotionMatchingController> MotionMatchingController::create()
{
    return std::make_shared<MotionMatchingController>(AK::Badge<MotionMatchingController> {});
}

MotionMatchingController::MotionMatchingController(AK::Badge<MotionMatchingController>)
{
}

void MotionMatchingController::initialize()
{
    Component::initialize();

    set_can_tick(true);

    if (path_point_container.expired())
    {
        path_point_container = Entity::create("Path Point Container");
        path_point_container.lock()->transform->set_parent(entity->transform);
    }

    for (u8 i = 0; i < mm_controller_debug_entity_pool; i++)
    {
        auto const e = Debug::draw_debug_sphere({0.0f, 0.0f, 0.0f}, 0.2f);
        e->name = "EXTRA_DEBUG_POINT_" + std::to_string(e->hashed_guid);
        e->get_component<Model>()->set_enabled(false);
        m_additional_debug_entity_pool.push(e);
    }
}

void MotionMatchingController::update_editor()
{
    Component::update_editor();

    // This is a hack for inability to reference components deserialized later than MotionMatchingController in initialize()
    if (m_first_update_pass)
    {
        m_cached_line.resize(m_line_points_num);
        if (entity->get_component<Curve>() == nullptr)
        {
            m_motion_matching_path = entity->add_component<Curve>(Curve::create());
            m_motion_matching_path->connection = PointsConnection::CatmullRom;
        }
        else
        {
            m_motion_matching_path = entity->get_component<Curve>();
        }

        m_first_update_pass = false;
    }

    draw_path();
}

void MotionMatchingController::awake()
{
    Component::awake();
    auto const sampler = AnimationEngine::get_instance()->get_motion_matching_sampler();
    m_sample_database_ref = std::make_shared<std::vector<Sample>>(sampler.lock()->sample_database);
    m_assets = Editor::Editor::get_instance()->get_assets(Editor::AssetType::Animation);
    path_point_container.lock()->transform->set_parent(nullptr);
    path_point_container.lock()->transform->set_position(entity->transform->get_position());
    path_point_container.lock()->transform->set_euler_angles(entity->transform->get_euler_angles());
    m_online_sample_rate = AnimationEngine::get_instance()->get_motion_matching_sampler().lock()->sample_rate;
    m_offline_average_root_step = AnimationEngine::get_instance()->get_motion_matching_sampler().lock()->offline_average_root_step;
    m_skinned_model_ref = entity->get_component<SkinnedModel>();
    generate_first_queue();
}

void MotionMatchingController::update()
{
    Component::update();

    get_nearest_point_on_curve_pos(entity->transform->get_position());
    sample_in_runtime();
}

#if EDITOR
void MotionMatchingController::draw_editor()
{
    Component::draw_editor();
    ImGui::SliderFloat("Path uniform scale", &path_scale, 0.0001f, 10.0f);

    if (m_motion_matching_path == nullptr)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
        ImGui::Text("Couldn't find motion matching path, probably there's no Curve component on this entity.");
        ImGui::PopStyleColor();
        return;
    }
}
#endif

void MotionMatchingController::draw_path()
{
    if (m_motion_matching_path == nullptr)
        return;

    if (m_motion_matching_path->curve.size() >= m_line_points_num)
    {
        Debug::log("MM path visualization error: Too many points on curve.", DebugType::Error);

        for (u32 i = 0; i < m_line_points_num; i++)
        {
            m_cached_line[i]->get_component<Model>()->set_enabled(false);
        }

        return;
    }

    if (!m_motion_matching_path->curve.empty())
    {
        m_motion_matching_path->points[0] = {0.0f, 0.0f};
        m_motion_matching_path->curve[0] = {0.0f, 0.0f};

        for (u32 i = 0; i < m_motion_matching_path->curve.size(); i++)
        {
            glm::vec2 point_pos = get_point_on_curve_by_index(i);
            glm::vec3 world_pos = editor_to_world_curve_pos(point_pos);

            if (m_cached_line[i] == nullptr)
            {
                auto const e = Debug::draw_debug_sphere({0.0f, 0.0f, 0.0f}, 0.07f);
                e->transform->set_parent(path_point_container.lock()->transform);
                e->name = "PATH_DEBUG_POINT_" + std::to_string(e->hashed_guid);
                e->get_component<Model>()->set_enabled(false);
                m_cached_line[i] = e;
            }

            m_cached_line[i]->transform->set_local_position(world_pos);
            m_cached_line[i]->get_component<Model>()->set_enabled(true);
        }

        for (u32 i = m_motion_matching_path->curve.size(); i < m_line_points_num; i++)
        {
            if (m_cached_line[i] == nullptr)
            {
                auto const e = Debug::draw_debug_sphere({0.0f, 0.0f, 0.0f}, 0.07f);
                e->transform->set_parent(path_point_container.lock()->transform);
                e->name = "PATH_DEBUG_POINT_" + std::to_string(e->hashed_guid);
                m_cached_line[i] = e;
            }

            m_cached_line[i]->get_component<Model>()->set_enabled(false);
        }
    }
}

void MotionMatchingController::sample_in_runtime()
{
    m_time += static_cast<float>(delta_time);

    if (m_time < m_online_sample_rate)
        return;

    m_time = 0.0f;

    Sample sample = {};

    // === FUTURE ===

    u32 current_nearest_point_on_curve_id = get_nearest_point_on_curve_id(entity->transform->get_position());
    glm::vec3 current_nearest_point_on_curve = get_nearest_point_on_curve_pos(entity->transform->get_position());

    // We need to estimate how many points do we want to skip
    u32 future_point_on_curve_id = current_nearest_point_on_curve_id;
    glm::vec3 future_point_on_curve = {};
    glm::vec3 good_ab = {};

    do
    {
        future_point_on_curve_id++;
        // POP FROM POOL
        auto const d = m_additional_debug_entity_pool.top();
        m_additional_debug_entity_pool.pop();
        d->transform->set_local_position(editor_to_world_curve_pos(get_point_on_curve_by_index(future_point_on_curve_id)));
        d->transform->set_parent(path_point_container.lock()->transform);

        ////////////////////////////////////////////////////////
        future_point_on_curve = d->transform->get_position();
        ////////////////////////////////////////////////////////

        // PUSH TO POOL
        d->transform->set_parent(nullptr);
        d->transform->set_local_position(glm::vec3(0.0f));
        m_additional_debug_entity_pool.push(d);

        // } while (glm::distance(future_point_on_curve, current_nearest_point_on_curve) < m_offline_average_root_step);
    } while (glm::distance(future_point_on_curve, current_nearest_point_on_curve) < 0.35f); // ADJUSTED!

    for (u32 i = 0; i <= feature_num; i++)
    {
        Feature feature = {};

        if (i == 0)
        {
            glm::vec3 ab = glm::normalize(future_point_on_curve - current_nearest_point_on_curve);
            feature.root_position = current_nearest_point_on_curve;

            // TODO: A better prediction?
            feature.left_foot_position = current_nearest_point_on_curve;
            feature.right_foot_position = current_nearest_point_on_curve;

            feature.facing_direction = ab;
            good_ab = ab;

            sample.current_feature = feature;

            Debug::draw_debug_sphere(current_nearest_point_on_curve, 0.2f, delta_time * 20.0f);
        }
        if (i > 0)
        {
            u32 const point_offset = future_point_on_curve_id - current_nearest_point_on_curve_id;
            u32 const selected_point_id = current_nearest_point_on_curve_id + point_offset * (i + 1);

            // POP FROM POOL
            auto const d = m_additional_debug_entity_pool.top();
            m_additional_debug_entity_pool.pop();
            d->transform->set_local_position(editor_to_world_curve_pos(get_point_on_curve_by_index(selected_point_id)));
            d->transform->set_parent(path_point_container.lock()->transform);

            ////////////////////////////////////////////////////////
            future_point_on_curve = d->transform->get_position();
            ////////////////////////////////////////////////////////

            // PUSH TO POOL
            d->transform->set_parent(nullptr);
            d->transform->set_local_position(glm::vec3(0.0f));
            m_additional_debug_entity_pool.push(d);

            glm::vec3 ab = glm::normalize(future_point_on_curve - current_nearest_point_on_curve);

            feature.root_position = future_point_on_curve;

            // TODO: A better prediction?
            feature.left_foot_position = future_point_on_curve;
            feature.right_foot_position = future_point_on_curve;

            feature.facing_direction = ab;

            sample.future_features.emplace_back(feature);

            Debug::draw_debug_sphere(future_point_on_curve, 0.2f, delta_time * 20.0f); // ???????????
        }
    }

    // === PAST & NOW ===
    Feature current_feature = {};

    m_skinned_model_ref.lock()->enable_root_motion = false;
    AnimationEngine::get_instance()->allow_animation_previews = false;

    glm::vec3 const root = MotionMatchingSampler::calculate_feature_position(m_skinned_model_ref.lock());
    std::vector<glm::vec3> feet = MotionMatchingSampler::calculate_feet_positions(m_skinned_model_ref.lock());
    glm::vec3 facing = MotionMatchingSampler::calculate_facing_direction(m_skinned_model_ref.lock());

    m_skinned_model_ref.lock()->enable_root_motion = true;
    AnimationEngine::get_instance()->allow_animation_previews = true;

    // We want it in a space consistent with future trajectory predictions
    auto const entity_pos = get_nearest_point_on_curve_pos(entity->transform->get_position());

    // These are feet offsets that require transforming to the space of 0-feature and its AB facing direction
    feet[0] = feet[0] - root;
    feet[0] = AK::Math::transform_to_new_space_z(feet[0], glm::vec3(0.0f, 0.0f, 1.0f));
    feet[0] += entity_pos;

    feet[1] = feet[1] - root;
    feet[1] = AK::Math::transform_to_new_space_z(feet[1], glm::vec3(0.0f, 0.0f, 1.0f));
    feet[1] += entity_pos;

    float radians = glm::radians(entity->transform->get_euler_angles().y);
    glm::quat rotation = glm::angleAxis(radians, glm::vec3(0.0f, 1.0f, 0.0f));
    facing = glm::normalize(rotation * facing);

    current_feature.root_position = entity_pos;
    current_feature.left_foot_position = feet[0];
    current_feature.right_foot_position = feet[1];
    current_feature.facing_direction = facing;

    // === UPDATING QUEUE ===
    m_past_feature_register.pop_front();
    m_past_feature_register.push_back(current_feature);

    for (u32 i = 0; i < feature_num; i++)
    {
        sample.past_features.emplace_back(m_past_feature_register[i]);
    }

    // === RELATIVIZING ===
    std::vector<Feature> features = MotionMatchingSampler::relativize_sample(sample);

    m_current_online_sample.past_features.clear();
    m_current_online_sample.future_features.clear();
    m_current_online_sample.current_feature = sample.current_feature;
    for (u32 i = 0; i < feature_num; i++)
    {
        m_current_online_sample.past_features.emplace_back(sample.past_features[i]);

        m_current_online_sample.future_features.emplace_back(sample.future_features[i]);
    }

    choose_best_sample(m_current_online_sample, good_ab);

    // Debug::clear();
    //
    // Debug::log("CURRENT ONLINE SAMPLE:");
    // for (i32 i = -feature_num; i <= feature_num; i++)
    // {
    //     Feature f = {};
    //
    //     if (i < 0)
    //         f = m_current_online_sample.past_features[i + feature_num];
    //
    //     if (i == 0)
    //         f = m_current_online_sample.current_feature;
    //
    //     if (i > 0)
    //         f = m_current_online_sample.future_features[i - 1];
    //
    //     Debug::log("      Feature " + std::to_string(i) + ": pos: " + std::to_string(f.root_position.x) + ", "
    //                + std::to_string(f.root_position.y) + ", " + std::to_string(f.root_position.z)
    //                + ", left foot position: " + std::to_string(f.left_foot_position.x) + ", " + std::to_string(f.left_foot_position.y)
    //                + ", " + std::to_string(f.left_foot_position.z) + ", right foot position: " + std::to_string(f.right_foot_position.x)
    //                + ", " + std::to_string(f.right_foot_position.y) + ", " + std::to_string(f.right_foot_position.z)
    //                + ", facing direction: " + std::to_string(f.facing_direction.x) + ", " + std::to_string(f.facing_direction.y) + ", "
    //                + std::to_string(f.facing_direction.z));
    // }
    //
    // Debug::log("");
    //
    // Debug::log("BEST DATABASE CHOSEN SAMPLE:");
    // for (i32 i = -feature_num; i <= feature_num; i++)
    // {
    //     Feature f = {};
    //
    //     if (i < 0)
    //         f = m_best_sample.past_features[i + feature_num];
    //
    //     if (i == 0)
    //         f = m_best_sample.current_feature;
    //
    //     if (i > 0)
    //         f = m_best_sample.future_features[i - 1];
    //
    //     Debug::log("      Feature " + std::to_string(i) + ": pos: " + std::to_string(f.root_position.x) + ", "
    //                + std::to_string(f.root_position.y) + ", " + std::to_string(f.root_position.z)
    //                + ", left foot position: " + std::to_string(f.left_foot_position.x) + ", " + std::to_string(f.left_foot_position.y)
    //                + ", " + std::to_string(f.left_foot_position.z) + ", right foot position: " + std::to_string(f.right_foot_position.x)
    //                + ", " + std::to_string(f.right_foot_position.y) + ", " + std::to_string(f.right_foot_position.z)
    //                + ", facing direction: " + std::to_string(f.facing_direction.x) + ", " + std::to_string(f.facing_direction.y) + ", "
    //                + std::to_string(f.facing_direction.z));
    // }
}

void MotionMatchingController::generate_first_queue()
{
    Feature feature = {};

    m_skinned_model_ref.lock()->enable_root_motion = false;
    AnimationEngine::get_instance()->allow_animation_previews = false;

    auto const root = MotionMatchingSampler::calculate_feature_position(m_skinned_model_ref.lock());
    auto feet = MotionMatchingSampler::calculate_feet_positions(m_skinned_model_ref.lock());
    auto facing = MotionMatchingSampler::calculate_facing_direction(m_skinned_model_ref.lock());

    m_skinned_model_ref.lock()->enable_root_motion = true;
    AnimationEngine::get_instance()->allow_animation_previews = true;

    // We want it in a space consistent with future trajectory predictions
    auto const entity_pos = get_nearest_point_on_curve_pos(entity->transform->get_position());

    // These are feet offsets that require transforming to the space of 0-feature and its AB facing direction
    feet[0] = feet[0] - root;
    feet[0] = AK::Math::transform_to_new_space_z(feet[0], glm::vec3(0.0f, 0.0f, 1.0f));
    feet[0] += entity_pos;

    feet[1] = feet[1] - root;
    feet[1] = AK::Math::transform_to_new_space_z(feet[1], glm::vec3(0.0f, 0.0f, 1.0f));
    feet[1] += entity_pos;

    float radians = glm::radians(entity->transform->get_euler_angles().y);
    glm::quat rotation = glm::angleAxis(radians, glm::vec3(0.0f, 1.0f, 0.0f));
    facing = glm::normalize(rotation * facing);
    feet[0] = rotation * feet[0];
    feet[1] = rotation * feet[1];

    for (u32 i = 0; i < feature_num + 1; i++)
    {
        feature.root_position = entity_pos;
        feature.left_foot_position = feet[0];
        feature.right_foot_position = feet[1];
        feature.facing_direction = facing;
        m_past_feature_register.push_back(feature);
    }
}

void MotionMatchingController::choose_best_sample(Sample const& online_sample, glm::vec3 const& realignment_vector)
{
    // TODO: Apply SIMD math here

    auto const start = std::chrono::high_resolution_clock::now();
    Sample current_online_sample = online_sample;
    float current_cost = FLT_MAX;

    // for (u32 i = 0; i < feature_num; i++)
    // {
    //     float multiplier = 1.0f * (feature_num - i);
    //     current_online_sample.past_features[i].root_position *= (multiplier * 1.0f);
    //     current_online_sample.past_features[i].facing_direction *= 1.0f;
    // }
    //
    // for (u32 i = 0; i < feature_num; i++)
    // {
    //     float multiplier = 1.0f * (i + 1);
    //     current_online_sample.future_features[i].root_position *= (multiplier * 1.0f);
    //     current_online_sample.future_features[i].facing_direction *= 1.0f;
    // }

    i32 best_sample_id = -1;

    std::random_device rd;
    std::mt19937 g(rd());

    // Shuffle the vector
    std::ranges::shuffle(*m_sample_database_ref, g);

    for (u32 sample_id = 0; sample_id < m_sample_database_ref->size(); sample_id++)
    {
        float vector_distance = 0.0f;
        auto const current_database_sample = m_sample_database_ref->at(sample_id);

        // vector_distance += glm::distance(current_online_sample.current_feature.left_foot_position,
        //                                  current_database_sample.current_feature.left_foot_position);
        // vector_distance += glm::distance(current_online_sample.current_feature.right_foot_position,
        //                                  current_database_sample.current_feature.right_foot_position);

        if (glm::distance(current_database_sample.past_features[0].root_position,
                          current_database_sample.future_features[feature_num - 1].root_position)
            < 3.0f)
            continue;

        for (u32 i = 0; i < feature_num; i++)
        {
            vector_distance += 1.0f
                             * glm::distance(current_online_sample.past_features[i].root_position,
                                             current_database_sample.past_features[i].root_position);
            // vector_distance += glm::distance(current_online_sample.past_features[i].left_foot_position,
            //                                  current_database_sample.past_features[i].left_foot_position);
            // vector_distance += glm::distance(current_online_sample.past_features[i].right_foot_position,
            //                                  current_database_sample.past_features[i].right_foot_position);
            vector_distance += 2.0f
                             * glm::distance(current_online_sample.past_features[i].facing_direction,
                                             current_database_sample.past_features[i].facing_direction);

            vector_distance += 2.0f
                             * glm::distance(current_online_sample.future_features[i].root_position,
                                             current_database_sample.future_features[i].root_position);
            // vector_distance += glm::distance(current_online_sample.future_features[i].left_foot_position,
            //                                  current_database_sample.future_features[i].left_foot_position);
            // vector_distance += glm::distance(current_online_sample.future_features[i].right_foot_position,
            //                                  current_database_sample.future_features[i].right_foot_position);
            vector_distance += 4.0f
                             * glm::distance(current_online_sample.future_features[i].facing_direction,
                                             current_database_sample.future_features[i].facing_direction);
        }

        // if (vector_distance >= current_cost)
        //     continue;

        if (vector_distance * 1.2f > current_cost)
            continue;

        current_cost = vector_distance;
        best_sample_id = sample_id;
    }

    auto const stop = std::chrono::high_resolution_clock::now();
    float const duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

    // if (current_cost * 2.74f > m_previous_cost)
    //     return;

    u32 const margin = 20;
    if (best_sample_id >= m_previous_best_sample_id - margin && best_sample_id <= m_previous_best_sample_id + margin)
        return;

    // if (m_cost_lock_counter >= 3)
    // {
    //     m_cost_lock_counter = 0;
    //     m_previous_cost = FLT_MAX;
    // }
    //
    // if (current_cost * 1.15f < m_previous_cost)
    // {
    //     m_previous_cost = current_cost;
    // }
    // else
    // {
    //     m_cost_lock_counter++;
    //     return;
    // }

    measure_execution_time(duration);
    measure_root_deviation();

    m_best_sample = m_sample_database_ref->at(best_sample_id);
    u32 const clip_id = m_best_sample.clip_id;
    float const clip_time = m_best_sample.clip_local_time;
    auto const asset = m_assets->at(clip_id).path;
    m_skinned_model_ref.lock()->anim_path = asset;
    m_skinned_model_ref.lock()->reprepare();
    m_skinned_model_ref.lock()->animation.current_time = clip_time;
    m_skinned_model_ref.lock()->calculate_bone_transform(&m_skinned_model_ref.lock()->animation.root_node, glm::mat4(1.0f));
    m_skinned_model_ref.lock()->align_animation_to_vector(realignment_vector);
    // Debug::log(std::to_string(good_ab.x) + ", " + std::to_string(good_ab.y) + ", " + std::to_string(good_ab.z));
    m_previous_best_sample_id = best_sample_id;
    // m_previous_cost = current_cost;
}

void MotionMatchingController::measure_root_deviation()
{
    m_accumulated_root_deviation +=
        glm::distance(entity->transform->get_position(), get_nearest_point_on_curve_pos(entity->transform->get_position()));
    m_root_deviation_measures_num++;

    if (get_nearest_point_on_curve_id(entity->transform->get_position()) >= m_motion_matching_path->curve.size() - 8)
    {
        float const average_deviation = m_accumulated_root_deviation / static_cast<float>(m_root_deviation_measures_num);
        Debug::log("Average root deviation: " + std::to_string(average_deviation));
    }
}

void MotionMatchingController::measure_execution_time(float current_execution_time)
{
    m_accumulated_execution_time += current_execution_time;
    m_execution_time_measures_num++;

    if (get_nearest_point_on_curve_id(entity->transform->get_position()) >= m_motion_matching_path->curve.size() - 8)
    {
        float const average_time = m_accumulated_execution_time / static_cast<float>(m_execution_time_measures_num);
        Debug::log("Average execution time: " + std::to_string(average_time / 1000.0f) + " ms");
    }
}

glm::vec3 MotionMatchingController::editor_to_world_curve_pos(glm::vec2 const& editor_pos)
{
    auto point_pos = AK::convert_2d_to_3d(editor_pos);

    // Likely ImPlot data stores data with X axis inverted, this fixes that
    point_pos.x = -point_pos.x;

    // ...and for some strange reason, the path in the world is rotated by -90 degrees. So we multiply it by quaternion representing 90 degrees rotation
    return glm::quat(0.707f, 0.0f, 0.707f, 0.0f) * point_pos * path_scale;
}

glm::vec2 MotionMatchingController::world_to_editor_curve_pos(glm::vec3 const& world_pos)
{
    // Undo the scale transformation
    glm::vec3 const unscaled_pos = world_pos / path_scale;

    // Undo the rotation transformation (apply inverse quaternion)
    glm::vec3 unrotated_pos = glm::quat(0.707f, 0.0f, -0.707f, 0.0f) * unscaled_pos;

    // Undo the x-axis inversion
    unrotated_pos.x = -unrotated_pos.x;

    return AK::convert_3d_to_2d(unrotated_pos);
}

glm::vec2 MotionMatchingController::get_point_on_curve_by_index(u32 const index)
{
    glm::vec2 previous_point_pos = {0.0f, 0.0f};
    u32 clamped_index = index;

    if (index > m_motion_matching_path->curve.size() - 1)
        clamped_index = m_motion_matching_path->curve.size() - 2;

    if (index > 0)
        previous_point_pos = m_motion_matching_path->curve[clamped_index - 1];

    return m_motion_matching_path->curve[clamped_index];
}

u32 MotionMatchingController::get_nearest_point_on_curve_id(glm::vec3 const& position)
{
    auto const d = m_additional_debug_entity_pool.top();
    m_additional_debug_entity_pool.pop();

    d->transform->set_parent(path_point_container.lock()->transform);
    d->transform->set_position(position);

    float nearest_distance = FLT_MAX;
    u32 nearest_point = 0;

    for (u32 i = 0; i < m_motion_matching_path->curve.size(); i++)
    {
        float const distance = glm::distance(editor_to_world_curve_pos(get_point_on_curve_by_index(i)), d->transform->get_local_position());
        if (distance < nearest_distance)
        {
            nearest_distance = distance;
            nearest_point = i;
        }
    }

    d->transform->set_position(glm::vec3(0.0f));
    d->transform->set_parent(nullptr);
    m_additional_debug_entity_pool.push(d);

    return nearest_point;
}

glm::vec3 MotionMatchingController::get_nearest_point_on_curve_pos(glm::vec3 const& position)
{
    u32 const nearest_point = get_nearest_point_on_curve_id(position);

    auto const d = m_additional_debug_entity_pool.top();
    m_additional_debug_entity_pool.pop();
    d->transform->set_local_position(editor_to_world_curve_pos(get_point_on_curve_by_index(nearest_point)));
    d->transform->set_parent(path_point_container.lock()->transform);

    //////////////////////////////////////////////////////////////
    glm::vec3 const world_position = d->transform->get_position();
    //////////////////////////////////////////////////////////////

    d->transform->set_local_position(glm::vec3(0.0f));
    d->transform->set_parent(nullptr);
    m_additional_debug_entity_pool.push(d);

    return world_position;
}
