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
            glm::vec2 point_pos = get_point_at_curve_by_index(i);
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
}

Sample MotionMatchingController::generate_first_sample()
{
    Sample sample = {};
    // Not implemented yet
    return sample;
}

glm::vec3 MotionMatchingController::editor_to_world_curve_pos(glm::vec2 const& editor_pos)
{
    auto point_pos = AK::convert_2d_to_3d(editor_pos);

    // Likely ImPlot data stores data with X axis inverted, this fixes that
    point_pos.x = -point_pos.x;

    // ...and for some strange reason, the path in the world is rotated by -90 degrees. So we multiply it by quaternion representing 90 degrees rotation
    return glm::quat(0.7f, 0.0f, 0.7f, 0.0f) * point_pos * path_scale;
}

glm::vec2 MotionMatchingController::get_point_at_curve_by_index(u32 const index)
{
    glm::vec2 previous_point_pos = {0.0f, 0.0f};

    if (index > 0)
        previous_point_pos = m_motion_matching_path->curve[index - 1];

    return m_motion_matching_path->curve[index];
}
