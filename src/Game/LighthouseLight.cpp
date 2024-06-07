#include <GLFW/glfw3.h>
#include <imgui.h>

#include "Entity.h"
#include "Input.h"
#include "LighthouseLight.h"

#include "LevelController.h"
#include "ResourceManager.h"
#include "imgui_extensions.h"

std::shared_ptr<LighthouseLight> LighthouseLight::create()
{
    return std::make_shared<LighthouseLight>(AK::Badge<LighthouseLight> {});
}

LighthouseLight::LighthouseLight(AK::Badge<LighthouseLight>)
{
}

void LighthouseLight::on_enabled()
{
    if (!m_sphere.expired())
        m_sphere.lock()->set_enabled(true);
}

void LighthouseLight::on_disabled()
{
    if (!m_sphere.expired())
        m_sphere.lock()->set_enabled(false);
}

void LighthouseLight::awake()
{
    set_can_tick(true);

    auto const standard_shader = ResourceManager::get_instance().load_shader("./res/shaders/lit.hlsl", "./res/shaders/lit.hlsl");
    auto const standard_material = Material::create(standard_shader);
    m_sphere = entity->add_component(Sphere::create(0.25f, 12, 12, "./res/textures/stone.jpg", standard_material));
}

void LighthouseLight::update()
{
    glm::vec2 const position = get_position();

    entity->transform->set_local_position(glm::vec3(position.x, 0.0f, position.y));

    if (spotlight.expired())
        return;

    auto const light_locked = spotlight.lock();

    float const light_beam_length = glm::length(entity->transform->get_position() - light_locked->entity->transform->get_position());
    float const aperture = glm::atan(spotlight_beam_width / light_beam_length);
    light_locked->cut_off = cos(aperture);
    light_locked->outer_cut_off = cos(aperture);
    light_locked->entity->transform->orient_towards(glm::vec3(position.x, 0.0f, position.y));
}

void LighthouseLight::draw_editor()
{
    ImGuiEx::draw_ptr("Spotlight", spotlight);
    ImGui::InputFloat("Beam width", &spotlight_beam_width);
}

void LighthouseLight::set_spot_light(std::shared_ptr<SpotLight> const& spot_light)
{
    spotlight = spot_light;
}

glm::vec2 LighthouseLight::get_position() const
{
    float const y = Input::input->get_mouse_position().y * LevelController::get_instance()->playfield_height
                  + LevelController::get_instance()->playfield_y_shift;
    float const x = Input::input->get_mouse_position().x
                  * (LevelController::get_instance()->playfield_width
                     - (LevelController::get_instance()->playfield_additional_width
                        * ((Input::input->get_mouse_position().y + LevelController::get_instance()->playfield_y_shift) + 1.0f) / 2.0f));

    return glm::vec2(x, y);
}
