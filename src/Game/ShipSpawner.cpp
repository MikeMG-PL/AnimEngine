#include "ShipSpawner.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <random>
#include <glm/gtc/random.hpp>

#include "Entity.h"
#include "Globals.h"
#include "ResourceManager.h"
#include "AK/AK.h"
#include "Collider2D.h"

std::shared_ptr<ShipSpawner> ShipSpawner::create()
{
    return std::make_shared<ShipSpawner>(AK::Badge<ShipSpawner> {});
}

std::shared_ptr<ShipSpawner> ShipSpawner::create(std::shared_ptr<LighthouseLight> const& light)
{
    auto ship_spawner = std::make_shared<ShipSpawner>(AK::Badge<ShipSpawner> {});
    ship_spawner->light = light;

    return ship_spawner;
}

ShipSpawner::ShipSpawner(AK::Badge<ShipSpawner>)
{
}

void ShipSpawner::awake()
{
    SpawnEvent s1 = {};
    s1.spawn_list.emplace_back(ShipType::FoodSmall);
    s1.spawn_type = SpawnType::Sequence;

    SpawnEvent s2 = {};
    s2.spawn_list.emplace_back(ShipType::FoodMedium);
    s2.spawn_type = SpawnType::Sequence;

    SpawnEvent s3 = {};
    s3.spawn_list.emplace_back(ShipType::FoodBig);
    s3.spawn_type = SpawnType::Sequence;

    SpawnEvent s4 = {};
    s4.spawn_list.emplace_back(ShipType::Tool);
    s4.spawn_type = SpawnType::Sequence;

    SpawnEvent s5 = {};
    s5.spawn_list.emplace_back(ShipType::Pirates);
    s5.spawn_type = SpawnType::Sequence;

    m_backup_spawn.emplace_back(s1);
    m_backup_spawn.emplace_back(s2);
    m_backup_spawn.emplace_back(s3);
    m_backup_spawn.emplace_back(s4);
    m_backup_spawn.emplace_back(s5);

    m_main_spawn = m_backup_spawn;

    auto const seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::ranges::shuffle(m_main_spawn, std::default_random_engine(seed));

    auto const path1 = entity->add_component<Path>(Path::create());
    path1->add_points({ { -5.0f, -2.5f }, { -5.0f, 1.7f } });

    auto const path2 = entity->add_component<Path>(Path::create());
    path2->add_points({ { 5.0f, -2.5f }, { 5.0f, 1.7f } });

    for (auto const& path : entity->get_components<Path>())
    {
        paths.emplace_back(path);
    }

    set_can_tick(true);
}

void ShipSpawner::update()
{
    prepare_for_spawn();
}

void ShipSpawner::draw_editor()
{
    u32 index = 0;
    for (auto const& ship : m_ships)
    {
        auto const ship_locked = ship.lock();
        ImGui::Text(("Ship " + std::to_string(index)).c_str());
        ImGui::SameLine();
        std::string type_string = ship_type_to_string(ship_locked->type);

        ImGui::Text(("Type " + type_string).c_str());
        if (!ship_locked->is_destroyed)
        {
            ImGui::SameLine();
            if (ImGui::Button(("Destroy##" + std::to_string(index)).c_str()))
            {
                ship_locked->destroy();
            }
        }

        index++;
    }

    ImGui::Separator();

    ImGui::Text(("Next ship counter " + std::to_string(m_spawn_warning_counter)).c_str());

    ImGui::Separator();

    if (ImGui::Button("Add Event"))
    {
        SpawnEvent new_element = {};
        m_backup_spawn.emplace_back(new_element);
    }

    for (u32 i = 0; i < m_backup_spawn.size(); i++) 
    {
        if (ImGui::Button(("Add Ship##" + std::to_string(i)).c_str()))
        {
            m_backup_spawn[i].spawn_list.emplace_back(ShipType::FoodSmall);
        }

        ImGui::SameLine();
        if (ImGui::Button(("Remove Event##" + std::to_string(i)).c_str()))
        {
            m_backup_spawn.erase(m_backup_spawn.begin() + i);
            i = i - 1;
            continue;
        }

        ImGui::SameLine();
        if (ImGui::BeginCombo(("##SpawnType" + std::to_string(i)).c_str(), spawn_type_to_string(m_backup_spawn[i].spawn_type).c_str()))
        {
            for (u32 j = static_cast<u32>(SpawnType::Sequence); j <= static_cast<u32>(SpawnType::Rapid); j++)
            {
                bool const is_selected = m_backup_spawn[i].spawn_type == static_cast<SpawnType>(j);

                if (ImGui::Selectable(spawn_type_to_string(static_cast<SpawnType>(j)).c_str(), is_selected))
                {
                    m_backup_spawn[i].spawn_type = static_cast<SpawnType>(j);
                }

                if (is_selected)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        for (u32 j = 0; j < m_backup_spawn[i].spawn_list.size(); j++)
        {
            ImGui::PushID(j);

            if (ImGui::BeginCombo(("##ShipType" + std::to_string(i) + std::to_string(j)).c_str(), ship_type_to_string(m_backup_spawn[i].spawn_list[j]).c_str()))
            {
                for (u32 k = static_cast<u32>(ShipType::FoodSmall); k <= static_cast<u32>(ShipType::Tool); k++)
                {
                    bool const is_selected = m_backup_spawn[i].spawn_list[j] == static_cast<ShipType>(k);

                    if (ImGui::Selectable(ship_type_to_string(static_cast<ShipType>(k)).c_str(), is_selected))
                    {
                        m_backup_spawn[i].spawn_list[j] = static_cast<ShipType>(k);
                    }

                    if (is_selected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::SameLine();
            if (ImGui::Button(("Remove ship##" + std::to_string(i) + std::to_string(j)).c_str()))
            {
                m_backup_spawn[i].spawn_list.erase(m_backup_spawn[i].spawn_list.begin() + j);
                j = j - 1;
                ImGui::PopID();
                continue;
            }

            if (m_backup_spawn[i].spawn_type == SpawnType::Rapid)
            {
                if (j != 0)
                {
                    ImGui::SameLine();
                    if (ImGui::ArrowButton(("##Up" + std::to_string(j)).c_str(), 2))
                    {
                        std::swap(m_backup_spawn[i].spawn_list[j], m_backup_spawn[i].spawn_list[j - 1]);
                    }
                }

                if (j != m_backup_spawn[i].spawn_list.size() - 1)
                {
                    ImGui::SameLine();
                    if (ImGui::ArrowButton(("##Down" + std::to_string(j)).c_str(), 3))
                    {
                        std::swap(m_backup_spawn[i].spawn_list[j], m_backup_spawn[i].spawn_list[j + 1]);
                    }
                }
            }

            ImGui::PopID();
        }
    }
}

void ShipSpawner::prepare_for_spawn()
{
    if (paths.size() == 0)
    {
        Debug::log("No available paths to create ships on!", DebugType::Warning);
        return;
    }

    if (!is_spawn_possible())
    {
        return;
    }

    if (m_spawn_warning_counter > 0.0f)
    {
        m_spawn_warning_counter -= delta_time;
        return;
    }

    if (m_warning_lights.size() != 0)
    {
        assert(m_warning_lights.size() == 1);

        if (m_ships.size() != 0)
        {
            if (glm::distance(find_nearest_ship(m_spawn_position), m_spawn_position) < minimum_spawn_distance)
            {
                // There is no room near the spawning point, delay until next spawn time
                m_spawn_warning_counter = spawn_warning_time;
                return;
            }
        }

        m_warning_lights[0].lock()->destroy_immediate();
        m_warning_lights.erase(m_warning_lights.begin());

        spawn_ship();

        return;
    }

    std::weak_ptr<Path> const path = paths[std::rand() % paths.size()];

    m_spawn_position = path.lock()->get_point_at(glm::linearRand(0.0f, 1.0f));

    m_spawn_warning_counter = spawn_warning_time;

    auto const warning = Entity::create("Warning");
    auto const warning_light_component = warning->add_component(SpotLight::create());

    warning_light_component->ambient = glm::vec3(1.0f);
    warning_light_component->diffuse = glm::vec3(1.0f);
    warning_light_component->specular = glm::vec3(1.0f);

    warning_light_component->linear = 0.0f;
    warning_light_component->quadratic = 0.0f;

    warning->transform->set_local_position({ m_spawn_position.x - glm::sign(m_spawn_position.x), 0.2f, m_spawn_position.y });
    warning->transform->set_euler_angles({ -90.0f, 0.0f, 0.0f });

    m_warning_lights.emplace_back(warning);
}

void ShipSpawner::spawn_ship()
{
    auto const standard_shader = ResourceManager::get_instance().load_shader("./res/shaders/lit.hlsl", "./res/shaders/lit.hlsl");
    auto const standard_material = Material::create(standard_shader);

    auto const ship = Entity::create("ship");
    ship->transform->set_local_position({ m_spawn_position.x, 0.0f, m_spawn_position.y });

    auto const ship_comp = ship->add_component(Ship::create(light.lock(), std::static_pointer_cast<ShipSpawner>(shared_from_this())));
    auto const collider = ship->add_component<Collider2D>(Collider2D::create({ 0.1f, 0.1f }));
    collider->set_is_trigger(true);
    ship_comp->on_ship_destroyed.attach(&ShipSpawner::remove_ship, shared_from_this());
    ship_comp->maximum_speed = LevelController::get_instance()->ships_speed;

    m_ships.emplace_back(ship_comp);

    if (m_backup_spawn.empty())
    {
        Debug::log("Ship spawn list is empty!", DebugType::Error);
        return;
    }

    if (m_main_spawn.back().spawn_list.empty())
    {
        m_main_spawn.pop_back();
    }

    if (m_main_spawn.empty())
    {
        m_main_spawn = m_backup_spawn;
        return;
    }

    SpawnEvent* being_spawn = &m_main_spawn.back();

    ship_comp->type = being_spawn->spawn_list.back();

    if (ship_comp->type == ShipType::FoodSmall)
    {
        ship->add_component(Model::create("./res/models/shipSmall/shipSmall.gltf", standard_material));
    }
    else if (ship_comp->type == ShipType::FoodMedium)
    {
        ship->add_component(Model::create("./res/models/shipMedium/shipMedium.gltf", standard_material));
    }
    else if (ship_comp->type == ShipType::FoodBig)
    {
        ship->add_component(Model::create("./res/models/shipBig/shipBig.gltf", standard_material));
    }
    else if (ship_comp->type == ShipType::Pirates)
    {
        ship->add_component(Model::create("./res/models/shipPirates/shipPirates.gltf", standard_material));
    }
    else if (ship_comp->type == ShipType::Tool)
    {
        ship->add_component(Model::create("./res/models/shipTool/shipTool.gltf", standard_material));
    }

    being_spawn->spawn_list.pop_back();
}

bool ShipSpawner::is_spawn_possible() const
{
    i32 number_of_ships = 0;

    for (auto const& ship : m_ships)
    {
        if (!ship.lock()->is_destroyed)
        {
            number_of_ships++;
        }
    }

    if (number_of_ships < LevelController::get_instance()->ships_limit)
    {
        return true;
    }

    return false;
}

std::string ShipSpawner::spawn_type_to_string(SpawnType const type)
{
    switch (type)
    {
    case SpawnType::Sequence:
        return "Sequence";

    case SpawnType::Immediate:
        return "Immediate";

    case SpawnType::Rapid:
        return "Rapid";

    default:
        return "Undefined spawn";
    }
}

void ShipSpawner::remove_ship(std::shared_ptr<Ship> const& ship_to_remove)
{
    AK::swap_and_erase(m_ships, ship_to_remove);
}

glm::vec2 ShipSpawner::find_nearest_non_pirate_ship(std::shared_ptr<Ship> const& center_ship) const
{
    auto const& nearest = m_ships[0];
    glm::vec2 const ship_position = AK::convert_3d_to_2d(center_ship->entity->transform->get_local_position());
    glm::vec2 nearest_position = AK::convert_3d_to_2d(nearest.lock()->entity->transform->get_local_position());
    float nearest_distance = glm::distance(ship_position, nearest_position);

    for (auto const& ship : m_ships)
    {
        auto const ship_locked = ship.lock();
        if (ship_locked == center_ship || ship_locked->type == ShipType::Pirates || ship_locked->is_destroyed)
        {
            continue;
        }

        glm::vec2 position = AK::convert_3d_to_2d(ship_locked->entity->transform->get_local_position());
        float const distance = glm::distance(ship_position, position);

        if (nearest_distance < distance)
        {
            nearest_distance = distance;
            nearest_position = position;
        }
    }

    return nearest_position;
}

glm::vec2 ShipSpawner::find_nearest_ship(glm::vec2 const center_position) const
{
    auto const& nearest = m_ships[0];
    glm::vec2 nearest_position = AK::convert_3d_to_2d(nearest.lock()->entity->transform->get_local_position());
    float nearest_distance = glm::distance(center_position, nearest_position);

    for (auto const& ship : m_ships)
    {
        auto const ship_locked = ship.lock();

        glm::vec2 position = AK::convert_3d_to_2d(ship_locked->entity->transform->get_local_position());
        float const distance = glm::distance(center_position, position);

        if (nearest_distance < distance)
        {
            nearest_distance = distance;
            nearest_position = position;
        }
    }

    return nearest_position;
}
