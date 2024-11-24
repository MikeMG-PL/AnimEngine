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

    if (entity->get_component<Curve>() == nullptr)
    {
        motion_matching_path = entity->add_component<Curve>(Curve::create());
        motion_matching_path->connection = PointsConnection::CatmullRom;
    }
    else
    {
        motion_matching_path = entity->get_component<Curve>();
    }

    auto const sampler = AnimationEngine::get_instance()->get_motion_matching_sampler();
    m_sample_database_ref = std::make_shared<std::vector<Sample>>(sampler.lock()->sample_database);

    for (u32 i = 0; i < m_line_points_num; i++)
    {
        auto const e = Debug::draw_debug_sphere({0.0f, 0.0f, 0.0f}, 0.07f);
        e->transform->set_parent(entity->transform);
        e->get_component<Model>()->set_enabled(false);
        m_cached_line.emplace_back(e->get_component<DebugDrawing>());
    }
}

void MotionMatchingController::update_editor()
{
    Component::update_editor();

    draw_path();
}

#if EDITOR
void MotionMatchingController::draw_editor()
{
    Component::draw_editor();
    ImGui::SliderFloat("Path uniform scale", &path_scale, 0.0001f, 10.0f);

    if (motion_matching_path == nullptr)
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
    glm::vec3 world_point_pos = entity->transform->get_position();

    if (motion_matching_path->curve.size() >= m_line_points_num)
    {
        Debug::log("MM path visualization error: Too many points on curve.", DebugType::Error);

        for (u32 i = 0; i < m_line_points_num; i++)
        {
            m_cached_line[i]->entity->get_component<Model>()->set_enabled(false);
        }

        return;
    }

    if (!motion_matching_path->curve.empty())
    {
        motion_matching_path->points[0] = {0.0f, 0.0f};
        motion_matching_path->curve[0] = {0.0f, 0.0f};

        for (u32 i = 0; i < motion_matching_path->curve.size(); i++)
        {
            glm::vec3 previous_point_pos = {0.0f, 0.0f, 0.0f};
            if (i > 0)
                previous_point_pos = AK::convert_2d_to_3d(motion_matching_path->curve[i - 1]);

            auto point_pos = AK::convert_2d_to_3d(motion_matching_path->curve[i]) - previous_point_pos;
            point_pos.x = -point_pos.x;
            world_point_pos += point_pos * path_scale;
            m_cached_line[i]->entity->transform->set_position(world_point_pos);
            m_cached_line[i]->entity->get_component<Model>()->set_enabled(true);
        }

        for (u32 i = motion_matching_path->curve.size(); i < m_line_points_num; i++)
        {
            m_cached_line[i]->entity->get_component<Model>()->set_enabled(false);
        }
    }
}
