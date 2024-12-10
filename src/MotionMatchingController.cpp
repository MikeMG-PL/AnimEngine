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
    path_point_container.lock()->transform->set_parent(nullptr);
    path_point_container.lock()->transform->set_position(entity->transform->get_position());
    path_point_container.lock()->transform->set_euler_angles(entity->transform->get_euler_angles());
    m_online_sample_rate = AnimationEngine::get_instance()->get_motion_matching_sampler().lock()->sample_rate;
    m_offline_average_root_step = AnimationEngine::get_instance()->get_motion_matching_sampler().lock()->offline_average_root_step;
    m_skinned_model_ref = entity->get_component<SkinnedModel>();

    // Set some default running clip to easily generate initial samples

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

    Sample sample_part_1 = {};
    Sample sample_part_2 = {};

    // === FUTURE ===

    // m_current_online_sample.future_features.clear();
    glm::vec3 ab = {};
    u32 first_step_on_curve_id = get_nearest_point_on_curve_id(entity->transform->get_position());
    glm::vec3 first_step_pos = get_nearest_point_on_curve_pos(entity->transform->get_position());
    glm::vec3 nearest_point_on_curve = first_step_pos;

    for (u32 i = 0; i <= feature_num; i++)
    {
        glm::vec3 const current_nearest_point_on_curve = first_step_pos;
        u32 next_point_on_curve_id = first_step_on_curve_id + 1;

        // POP FROM POOL
        auto const d = m_additional_debug_entity_pool.top();
        m_additional_debug_entity_pool.pop();
        d->transform->set_local_position(editor_to_world_curve_pos(get_point_on_curve_by_index(next_point_on_curve_id)));
        d->transform->set_parent(path_point_container.lock()->transform);

        ////////////////////////////////////////////////////////
        glm::vec3 next_point_pos = d->transform->get_position();
        ////////////////////////////////////////////////////////

        // PUSH TO POOL
        d->transform->set_parent(nullptr);
        d->transform->set_local_position(glm::vec3(0.0f));
        m_additional_debug_entity_pool.push(d);

        float distance = glm::distance(first_step_pos, next_point_pos);
        float last_step = 0.0f;
        while (distance < m_offline_average_root_step)
        {
            glm::vec3 const a = next_point_pos;

            first_step_on_curve_id = next_point_on_curve_id;
            next_point_on_curve_id++;

            // POP FROM POOL
            auto const d = m_additional_debug_entity_pool.top();
            m_additional_debug_entity_pool.pop();
            d->transform->set_local_position(editor_to_world_curve_pos(get_point_on_curve_by_index(next_point_on_curve_id)));
            d->transform->set_parent(path_point_container.lock()->transform);

            /////////////////////////////////////////////////
            glm::vec3 const b = d->transform->get_position();
            /////////////////////////////////////////////////

            // PUSH TO POOL
            d->transform->set_parent(nullptr);
            d->transform->set_local_position(glm::vec3(0.0f));
            m_additional_debug_entity_pool.push(d);

            last_step = glm::distance(a, b);
            distance += last_step;

            next_point_pos = b;
            first_step_pos = a;
        }

        ab = normalize(next_point_pos - first_step_pos);
        Feature feature = {};

        if (i == 0)
        {
            feature.root_position = current_nearest_point_on_curve;

            // TODO: A better prediction?
            feature.left_foot_position = current_nearest_point_on_curve;
            feature.right_foot_position = current_nearest_point_on_curve;

            feature.facing_direction = ab;

            sample_part_2.current_feature = feature;
        }
        if (i > 0)
        {
            feature.root_position = first_step_pos;

            // TODO: A better prediction?
            feature.left_foot_position = first_step_pos;
            feature.right_foot_position = first_step_pos;

            feature.facing_direction = ab;

            sample_part_2.future_features.emplace_back(feature);

            Debug::draw_debug_sphere(first_step_pos, 0.2f, delta_time * 20.0f);
        }
    }

    Feature const mock_past = {};

    for (u32 j = 0; j < feature_num; j++)
        sample_part_2.past_features.emplace_back(mock_past);

    // === RELATIVIZING FUTURE ===
    std::vector<Feature> features_part_2 = MotionMatchingSampler::relativize_sample(sample_part_2);

    // === PAST ===

    m_skinned_model_ref.lock()->enable_root_motion = false;
    AnimationEngine::get_instance()->allow_animation_previews = false;

    float const current_anim_time = m_skinned_model_ref.lock()->animation.current_time;
    // m_current_online_sample.past_features.clear();

    for (i32 j = -feature_num; j <= 0; j++)
    {
        float const step = static_cast<float>(j) * m_online_sample_rate * 1000.0f;

        m_skinned_model_ref.lock()->animation.current_time = current_anim_time + step;
        m_skinned_model_ref.lock()->calculate_bone_transform(&m_skinned_model_ref.lock()->animation.root_node, glm::mat4(1.0f));

        glm::vec3 feature_pos = MotionMatchingSampler::calculate_feature_position(m_skinned_model_ref.lock());
        glm::vec3 facing_direction = MotionMatchingSampler::calculate_facing_direction(m_skinned_model_ref.lock());
        std::vector<glm::vec3> feet_positions = MotionMatchingSampler::calculate_feet_positions(m_skinned_model_ref.lock());

        Feature feature = {};
        feature.root_position = feature_pos;
        feature.facing_direction = facing_direction;
        feature.left_foot_position = feet_positions[0];
        feature.right_foot_position = feet_positions[1];

        if (j < 0)
            sample_part_1.past_features.emplace_back(feature);

        if (j == 0)
            sample_part_1.current_feature = feature;
    }

    Feature const mock_future = {};

    for (u32 j = 0; j < feature_num; j++)
        sample_part_1.future_features.emplace_back(mock_future);

    m_skinned_model_ref.lock()->animation.current_time = current_anim_time;
    m_skinned_model_ref.lock()->calculate_bone_transform(&m_skinned_model_ref.lock()->animation.root_node, glm::mat4(1.0f));

    m_skinned_model_ref.lock()->enable_root_motion = true;
    AnimationEngine::get_instance()->allow_animation_previews = true;

    // === RELATIVIZING PAST ===
    std::vector<Feature> features_part_1 = MotionMatchingSampler::relativize_sample(sample_part_1);

    // // rewrite xssfasddddddddddddddddd!
    // for (i32 j = -feature_num; j <= feature_num; j++)
    // {
    //     Feature const f = features_part_1[j + feature_num];
    //
    //     Debug::log("      Feature " + std::to_string(j) + ": pos: " + std::to_string(f.root_position.x) + ", "
    //                + std::to_string(f.root_position.y) + ", " + std::to_string(f.root_position.z)
    //                + ", left foot position: " + std::to_string(f.left_foot_position.x) + ", " + std::to_string(f.left_foot_position.y)
    //                + ", " + std::to_string(f.left_foot_position.z) + ", right foot position: " + std::to_string(f.right_foot_position.x)
    //                + ", " + std::to_string(f.right_foot_position.y) + ", " + std::to_string(f.right_foot_position.z)
    //                + ", facing direction: " + std::to_string(f.facing_direction.x) + ", " + std::to_string(f.facing_direction.y) + ", "
    //                + std::to_string(f.facing_direction.z));
    // }

    m_current_online_sample.past_features.clear();
    m_current_online_sample.future_features.clear();
    m_current_online_sample.current_feature = sample_part_1.current_feature;
    for (u32 i = 0; i < feature_num; i++)
    {
        m_current_online_sample.past_features.emplace_back(sample_part_1.past_features[i]);
        m_current_online_sample.future_features.emplace_back(sample_part_2.future_features[i]);
    }

    for (i32 j = -feature_num; j <= feature_num; j++)
    {
        Feature f = {};

        if (j < 0)
            f = m_current_online_sample.past_features[j + feature_num];

        if (j == 0)
            f = m_current_online_sample.current_feature;

        if (j > 0)
            f = m_current_online_sample.future_features[j - 1];

        Debug::log("      Feature " + std::to_string(j) + ": pos: " + std::to_string(f.root_position.x) + ", "
                   + std::to_string(f.root_position.y) + ", " + std::to_string(f.root_position.z)
                   + ", left foot position: " + std::to_string(f.left_foot_position.x) + ", " + std::to_string(f.left_foot_position.y)
                   + ", " + std::to_string(f.left_foot_position.z) + ", right foot position: " + std::to_string(f.right_foot_position.x)
                   + ", " + std::to_string(f.right_foot_position.y) + ", " + std::to_string(f.right_foot_position.z)
                   + ", facing direction: " + std::to_string(f.facing_direction.x) + ", " + std::to_string(f.facing_direction.y) + ", "
                   + std::to_string(f.facing_direction.z));
    }

    // === UPDATING QUEUE ===

    // Feature feature = {};
    //
    // m_skinned_model_ref.lock()->enable_root_motion = false;
    // AnimationEngine::get_instance()->allow_animation_previews = false;
    //
    // glm::vec3 const root = MotionMatchingSampler::calculate_feature_position(m_skinned_model_ref.lock());
    // std::vector<glm::vec3> const feet = MotionMatchingSampler::calculate_feet_positions(m_skinned_model_ref.lock());
    // glm::vec3 const facing = MotionMatchingSampler::calculate_facing_direction(m_skinned_model_ref.lock());
    //
    // m_skinned_model_ref.lock()->enable_root_motion = true;
    // AnimationEngine::get_instance()->allow_animation_previews = true;
    //
    // feature.root_position = root;
    // feature.left_foot_position = feet[0];
    // feature.right_foot_position = feet[1];
    // feature.facing_direction = facing;
    //
    // m_past_feature_register.pop_front();
    // m_past_feature_register.emplace_back(feature, root);

    //
    // m_current_online_sample.past_features[m_current_online_sample.past_features.size() - 1] = cached_current_feature;
}

Sample MotionMatchingController::generate_first_sample()
{
    // ??????????????????

    Sample sample = {};
    Feature feature = {};

    m_skinned_model_ref.lock()->enable_root_motion = false;
    AnimationEngine::get_instance()->allow_animation_previews = false;

    auto const current_curve_pos = get_nearest_point_on_curve_pos(entity->transform->get_position());
    auto const current_root_pos = MotionMatchingSampler::calculate_feature_position(m_skinned_model_ref.lock());

    for (i32 i = -feature_num; i < 0; i++)
    {
        auto const root = current_curve_pos;
        auto const feet = MotionMatchingSampler::calculate_feet_positions(m_skinned_model_ref.lock());
        auto const facing = MotionMatchingSampler::calculate_facing_direction(m_skinned_model_ref.lock());

        feature.root_position = root;
        feature.left_foot_position = feet[0] - current_root_pos + current_curve_pos;
        feature.right_foot_position = feet[1] - current_root_pos + current_curve_pos;
        feature.facing_direction = facing;
        sample.past_features.emplace_back(feature);

        m_past_feature_register.emplace_back(feature, current_root_pos);
    }

    m_skinned_model_ref.lock()->enable_root_motion = true;
    AnimationEngine::get_instance()->allow_animation_previews = true;

    // // Feature -3
    // feature.root_position = glm::vec3(0.0f);
    // feature.left_foot_position = glm::vec3(0.0f);
    // feature.right_foot_position = glm::vec3(0.0f);
    // feature.facing_direction = glm::vec3(0.0f, 0.0f, 1.0f);
    // sample.past_features.emplace_back(feature);
    //
    // // Feature -2
    // feature.root_position = glm::vec3(0.0f);
    // feature.left_foot_position = glm::vec3(0.0f);
    // feature.right_foot_position = glm::vec3(0.0f);
    // feature.facing_direction = glm::vec3(0.0f, 0.0f, 1.0f);
    // sample.past_features.emplace_back(feature);
    //
    // // Feature -1
    // feature.root_position = glm::vec3(0.0f);
    // feature.left_foot_position = glm::vec3(0.0f);
    // feature.right_foot_position = glm::vec3(0.0f);
    // feature.facing_direction = glm::vec3(0.0f, 0.0f, 1.0f);
    // sample.past_features.emplace_back(feature);

    // other ones

    // // Feature -3
    // feature.root_position = {-0.018072f, 0.004021f, -1.064458f};
    // feature.left_foot_position = {0.019885f, -0.074155f, 0.419956f};
    // feature.right_foot_position = {0.180776f, -0.074155f, 0.714577f};
    // feature.facing_direction = {0.177852f, 0.000000f, 0.984057f};
    // sample.past_features.emplace_back(feature);
    //
    // // Feature -2
    // feature.root_position = {0.000803f, -0.074155f, -0.731986f};
    // feature.left_foot_position = {-0.008258f, -0.074155f, -0.830488f};
    // feature.right_foot_position = {-0.029776f, -0.074155f, -1.474204f};
    // feature.facing_direction = {0.141448f, 0.000000f, 0.989946f};
    // sample.past_features.emplace_back(feature);
    //
    // // Feature -1
    // feature.root_position = {0.001385f, -0.066157f, -0.383387f};
    // feature.left_foot_position = {-0.134224f, -0.074155f, -0.710428f};
    // feature.right_foot_position = {0.005454f, -0.074155f, -0.917133f};
    // feature.facing_direction = {0.081737f, 0.000000f, 0.996654f};
    // sample.past_features.emplace_back(feature);

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
