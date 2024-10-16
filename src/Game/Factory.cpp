#include "Factory.h"

#include "Entity.h"
#include "Game/LevelController.h"
#include "Model.h"
#include "Player.h"
#include "ResourceManager.h"
#include "SceneSerializer.h"

#if EDITOR
#include "imgui_extensions.h"
#endif

std::shared_ptr<Factory> Factory::create()
{
    return std::make_shared<Factory>(AK::Badge<Factory> {});
}

Factory::Factory(AK::Badge<Factory>)
{
}

void Factory::awake()
{
    set_can_tick(true);
}

bool Factory::interact()
{
    if (Player::get_instance()->packages <= 0)
        return false;

    if (type == FactoryType::Generator)
    {
        if (Player::get_instance()->flash >= m_max_flash_count)
        {
            return false;
        }

        Player::get_instance()->packages -= 1;
        Player::get_instance()->flash += 1;
        update_lights();
        LevelController::get_instance()->check_tutorial_progress(TutorialProgressAction::GeneratorFueled);
        auto const particle = SceneSerializer::load_prefab("GeneratorUpgrade");
        particle->transform->set_position(entity->transform->get_position());
    }
    else if (type == FactoryType::Workshop)
    {
        if (Player::get_instance()->lighthouse_level >= LevelController::get_instance()->maximum_lighthouse_level - 1)
        {
            return false;
        }
        else
        {
            Player::get_instance()->upgrade_lighthouse();
            LevelController::get_instance()->check_tutorial_progress(TutorialProgressAction::WorkshopUpgraded);

            auto const particle = SceneSerializer::load_prefab("WorkshopUpgrade");
            particle->transform->set_position(entity->transform->get_position());

            Player::get_instance()->packages -= 1;
        }
    }

    return true;
}

#if EDITOR
void Factory::draw_editor()
{
    Component::draw_editor();

    std::array const factory_types = {"Generator", "Workshop"};
    i32 current_item_index = static_cast<i32>(type);
    if (ImGui::Combo("Factory Type", &current_item_index, factory_types.data(), factory_types.size()))
    {
        type = static_cast<FactoryType>(current_item_index);
    }
}
#endif

void Factory::set_type(FactoryType const type)
{
    this->type = type;

    if (!model.expired())
    {
        model.lock()->destroy_immediate();
    }

    auto const standard_shader = ResourceManager::get_instance().load_shader("./res/shaders/lit.hlsl", "./res/shaders/lit.hlsl");
    auto const standard_material = Material::create(standard_shader);

    if (type == FactoryType::Generator)
    {
        model = entity->add_component<Model>(Model::create("./res/models/hubert/floor.gltf", standard_material));
        entity->transform->set_local_scale({0.05f, 0.1f, 0.05f});
    }
    else if (type == FactoryType::Workshop)
    {
        model = entity->add_component<Model>(Model::create("./res/models/lighthouseOld/lighthouse.gltf", standard_material));
        entity->transform->set_local_scale({1.0f, 1.0f, 1.0f});
    }
}

void Factory::turn_off_lights() const
{
    if (type == FactoryType::Generator)
    {
        for (size_t i = 0; i < lights.size(); i++)
        {
            lights[i].lock()->set_enabled(false);
        }
    }
}

void Factory::update_lights() const
{
    if (type == FactoryType::Generator)
    {
        for (u32 i = 0; i < lights.size(); i++)
        {
            if (i < Player::get_instance()->flash)
            {
                lights[i].lock()->set_burn_out(false);
                lights[i].lock()->set_enabled(true);
            }
            else
            {
                lights[i].lock()->set_burn_out(true);
            }
        }
    }
    else
    {
        Debug::log("Something wants to update lights in Workshop where there are no lights!", DebugType::Warning);
    }
}

void Factory::set_glowing(bool const is_glowing) const
{
    auto const drawable = entity->get_component<Drawable>();
    drawable->set_glowing(is_glowing);
}

i32 Factory::get_max_flash_count() const
{
    return m_max_flash_count;
}
