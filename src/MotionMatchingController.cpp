#include "MotionMatchingController.h"

#include "AK/AK.h"
#include "AnimationEngine.h"
#include "Entity.h"

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

    auto const sampler = AnimationEngine::get_instance()->get_motion_matching_sampler();
    m_sample_database_ref = std::make_shared<std::vector<Sample>>(sampler.lock()->sample_database);

    if (path_point_container.expired())
    {
        path_point_container = Entity::create("Path Point Container");
        path_point_container.lock()->transform->set_parent(entity->transform);
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
    path_point_container.lock()->transform->set_parent(nullptr);
    path_point_container.lock()->transform->set_position(entity->transform->get_position());
    path_point_container.lock()->transform->set_euler_angles(entity->transform->get_euler_angles());
    m_online_sample_rate = AnimationEngine::get_instance()->get_motion_matching_sampler().lock()->sample_rate;
    m_offline_average_root_step = AnimationEngine::get_instance()->get_motion_matching_sampler().lock()->offline_average_root_step;

    // Set some default running clip to easily generate initial samples
    // m_skinned_model_ref = entity->get_component<SkinnedModel>();
    // m_skinned_model_ref.lock()->anim_path = "./res/anims/conv16_36_Anim.gltf";
    // m_skinned_model_ref.lock()->reprepare();
    // m_skinned_model_ref.lock()->animation.current_time = 2000.0f;

    m_current_online_sample = generate_first_sample();
}

void MotionMatchingController::update()
{
    Component::update();

    // auto const point = get_point_on_curve_by_index(80);
    // auto const x = editor_to_world_curve_pos(point);
    // auto const y = world_to_editor_curve_pos(x);
    // auto const e = Debug::draw_debug_box(x, {0.0f, 0.0f, 0.0f}, {0.25f, 0.25f, 0.25f}, delta_time * 2.0f);
    // e->transform->set_parent(path_point_container.lock()->transform);

    get_nearest_point_on_curve_pos(entity->transform->get_position());
    // sample_in_runtime();
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

    // === MOCK PAST ===
    for (u32 i = 0; i < feature_num; i++)
    {
        m_current_online_sample.past_features[i] = {};
    }

    // === NOW & FUTURE ===

    for (u32 i = 0; i <= feature_num; i++)
    {
        u32 first_step_on_curve_id = get_nearest_point_on_curve_id(entity->transform->get_position());
        glm::vec3 first_step_pos = get_nearest_point_on_curve_pos(entity->transform->get_position());

        u32 next_point_on_curve_id = first_step_on_curve_id + 1;

        auto const d = Debug::draw_debug_sphere(editor_to_world_curve_pos(get_point_on_curve_by_index(next_point_on_curve_id)), 0.2f,
                                                delta_time * 2.0f);
        d->transform->set_parent(path_point_container.lock()->transform);
        glm::vec3 next_point_pos = d->transform->get_position();

        float distance = glm::distance(first_step_pos, next_point_pos);
        float last_step = 0.0f;
        while (distance < m_offline_average_root_step)
        {
            glm::vec3 const a = next_point_pos;

            first_step_on_curve_id = next_point_on_curve_id;
            next_point_on_curve_id++;

            auto const d =
                Debug::draw_debug_sphere(editor_to_world_curve_pos(get_point_on_curve_by_index(next_point_on_curve_id)), 0.2f, 0.0001f);
            d->transform->set_parent(path_point_container.lock()->transform);
            glm::vec3 const b = d->transform->get_position();

            last_step = glm::distance(a, b);
            distance += last_step;

            next_point_pos = b;
            first_step_pos = a;
        }

        // When you draw it you can conclude that the point on AB vector where new sampled root should be is calculated with the formula:
        float const new_root_pos_between_ab = m_offline_average_root_step - distance + last_step;

        glm::vec3 const ab = normalize(next_point_pos - first_step_pos);
        glm::vec3 const world_root_next_pos = first_step_pos + ab * new_root_pos_between_ab;

        Debug::draw_debug_sphere(world_root_next_pos, 0.4f, 0.0001f);

        Feature feature = {};

        if (i == 0)
        {
            feature.root_position = first_step_pos;

            // TODO: A better prediction?
            feature.left_foot_position = world_root_next_pos;
            feature.right_foot_position = world_root_next_pos;

            feature.facing_direction = ab;

            m_current_online_sample.current_feature = feature;
        }
        else
        {
            feature.root_position = world_root_next_pos;

            // TODO: A better prediction?
            feature.left_foot_position = world_root_next_pos;
            feature.right_foot_position = world_root_next_pos;

            feature.facing_direction = ab;

            m_current_online_sample.future_features.emplace_back(feature);
        }
    }

    // === RELATIVIZING ===

    // Some relativizing here
    // std::vector<Feature> features = relativize_sample(sample);

    // === PAST ===
    for (u32 i = 0; i < feature_num - 1; i++)
    {
        m_current_online_sample.past_features[i] = m_current_online_sample.past_features[i - 1];
    }

    m_current_online_sample.past_features[m_current_online_sample.past_features.size() - 1] = m_current_online_sample.current_feature;
}

Sample MotionMatchingController::generate_first_sample()
{
    Sample sample = {};
    Feature feature = {};

    // Feature -3
    feature.root_position = {-0.018072f, 0.004021f, -1.064458f};
    feature.left_foot_position = {0.019885f, -0.074155f, 0.419956f};
    feature.right_foot_position = {0.180776f, -0.074155f, 0.714577f};
    feature.facing_direction = {0.177852f, 0.000000f, 0.984057f};
    sample.past_features.emplace_back(feature);

    // Feature -2
    feature.root_position = {0.000803f, -0.074155f, -0.731986f};
    feature.left_foot_position = {-0.008258f, -0.074155f, -0.830488f};
    feature.right_foot_position = {-0.029776f, -0.074155f, -1.474204f};
    feature.facing_direction = {0.141448f, 0.000000f, 0.989946f};
    sample.past_features.emplace_back(feature);

    // Feature -1
    feature.root_position = {0.001385f, -0.066157f, -0.383387f};
    feature.left_foot_position = {-0.134224f, -0.074155f, -0.710428f};
    feature.right_foot_position = {0.005454f, -0.074155f, -0.917133f};
    feature.facing_direction = {0.081737f, 0.000000f, 0.996654f};
    sample.past_features.emplace_back(feature);

    // Feature 0
    feature.root_position = {0.000000f, 0.000000f, 0.000000f};
    feature.left_foot_position = {-0.116665f, -0.074155f, -0.629486f};
    feature.right_foot_position = {0.045025f, -0.074155f, -0.300803f};
    feature.facing_direction = {0.000000f, 0.000000f, 1.000000f};
    sample.current_feature = feature;

    // Feature 1
    feature.root_position = {-0.003410f, -0.049177f, 0.356194f};
    feature.left_foot_position = {-0.022586f, -0.074155f, -0.432611f};
    feature.right_foot_position = {0.078490f, -0.074155f, 0.282952f};
    feature.facing_direction = {-0.016264f, 0.000000f, 0.999868f};
    sample.future_features.emplace_back(feature);

    // Feature 2
    feature.root_position = {-0.019107f, -0.074155f, 0.699193f};
    feature.left_foot_position = {0.015285f, -0.074155f, -0.097925f};
    feature.right_foot_position = {0.125771f, -0.074155f, 0.604311f};
    feature.facing_direction = {-0.009463f, 0.000000f, 0.999955f};
    sample.future_features.emplace_back(feature);

    // Feature 3
    feature.root_position = {-0.001487f, -0.060330f, 1.063376f};
    feature.left_foot_position = {0.019885f, -0.074155f, 0.419956f};
    feature.right_foot_position = {0.180776f, -0.074155f, 0.714577f};
    feature.facing_direction = {0.026722f, 0.000000f, 0.999643f};
    sample.future_features.emplace_back(feature);

    return sample;
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

    if (index > 0)
        previous_point_pos = m_motion_matching_path->curve[index - 1];

    return m_motion_matching_path->curve[index];
}

u32 MotionMatchingController::get_nearest_point_on_curve_id(glm::vec3 const& position)
{
    auto const e = Entity::create_internal(".");
    e->transform->set_parent(path_point_container.lock()->transform);
    e->transform->set_position(position);

    float nearest_distance = FLT_MAX;
    u32 nearest_point = 0;

    for (u32 i = 0; i < m_motion_matching_path->curve.size(); i++)
    {
        float const distance = glm::distance(editor_to_world_curve_pos(get_point_on_curve_by_index(i)), e->transform->get_local_position());
        if (distance < nearest_distance)
        {
            nearest_distance = distance;
            nearest_point = i;
        }
    }

    return nearest_point;
}

glm::vec3 MotionMatchingController::get_nearest_point_on_curve_pos(glm::vec3 const& position)
{
    u32 const nearest_point = get_nearest_point_on_curve_id(position);

    auto const d = Debug::draw_debug_sphere(editor_to_world_curve_pos(get_point_on_curve_by_index(nearest_point)), 0.2f, delta_time * 2.0f);
    d->transform->set_parent(path_point_container.lock()->transform);
    glm::vec3 const world_position = d->transform->get_position();

    Debug::log(std::to_string(world_position.x) + ", " + std::to_string(world_position.y) + ", " + std::to_string(world_position.z));

    return world_position;
}
