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

    draw_path();
}
#endif

void MotionMatchingController::draw_path()
{
    glm::vec3 world_point_pos = entity->transform->get_position();

    // TODO: Handle rotation

    for (u32 i = 0; i < motion_matching_path->curve.size(); i++)
    {
        auto const point_pos = AK::convert_2d_to_3d(motion_matching_path->curve[i]);
        world_point_pos += point_pos * path_scale;
        //Debug::draw_debug_sphere(world_point_pos, 0.1f, delta_time * 2.0f);
    }
}
