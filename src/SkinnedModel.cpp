#include "SkinnedModel.h"

#include "AK/AK.h"
#include "AK/Math.h"
#include "AK/Types.h"
#include "AnimationEngine.h"
#include "ConstantBufferTypes.h"
#include "Entity.h"
#include "Globals.h"
#include "Mesh.h"
#include "MeshFactory.h"
#include "Renderer.h"
#include "RendererDX11.h"
#include "ResourceManager.h"
#include "Texture.h"
#include "Vertex.h"
#include "glm/gtx/matrix_decompose.hpp"

#include <filesystem>
#include <iostream>
#include <map>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#if EDITOR
#include <imgui.h>
#include <imgui_stdlib.h>
#endif

std::shared_ptr<SkinnedModel> SkinnedModel::create()
{
    auto model = std::make_shared<SkinnedModel>(AK::Badge<SkinnedModel> {}, default_material);
    model->prepare();

    return model;
}

std::shared_ptr<SkinnedModel> SkinnedModel::create(std::string const& model_path, std::string const& anim_path,
                                                   std::shared_ptr<Material> const& material)
{
    auto model = std::make_shared<SkinnedModel>(AK::Badge<SkinnedModel> {}, model_path, anim_path, material);
    model->prepare();

    return model;
}

std::shared_ptr<SkinnedModel> SkinnedModel::create(std::shared_ptr<Material> const& material)
{
    auto model = std::make_shared<SkinnedModel>(AK::Badge<SkinnedModel> {}, material);
    model->prepare();

    return model;
}

std::shared_ptr<SkinnedModel> SkinnedModel::create(std::shared_ptr<Mesh> mesh, std::shared_ptr<Material> const& material)
{
    auto model = std::make_shared<SkinnedModel>(AK::Badge<SkinnedModel> {}, material);

    model->m_meshes.emplace_back(mesh);

    return model;
}

SkinnedModel::SkinnedModel(AK::Badge<SkinnedModel>, std::string const& model_path, std::string const& anim_path,
                           std::shared_ptr<Material> const& material)
    : Drawable(material), model_path(model_path), anim_path(anim_path)
{
}

SkinnedModel::SkinnedModel(AK::Badge<SkinnedModel>, std::shared_ptr<Material> const& material) : Drawable(material)
{
}

#if EDITOR
void SkinnedModel::draw_editor()
{
    Drawable::draw_editor();

    ImGui::InputText("SkinnedModel Path", &model_path);

    if (ImGui::IsItemDeactivatedAfterEdit())
    {
        reprepare();
    }

    ImGui::InputText("Animation Path", &anim_path);

    if (ImGui::IsItemDeactivatedAfterEdit())
    {
        reprepare();
    }

    if (ImGui::Button("Readjust anim // test"))
    {
        align_animation_to_vector(glm::vec3(0, 0, 1));
    }

    // if (ImGui::Button("Switch blending"))
    // {
    //     if (m_blend_between_clips)
    //         stop_blending();
    //     else
    //         start_blending();
    // }

    ImGui::SliderFloat("Blend Alpha", &blend_value, 0.0f, 1.0f);

    ImGui::Checkbox("Enable root motion", &enable_root_motion);

    // Choose rasterizer draw mode for individual model
    std::array const draw_type_items = {"Default", "Wireframe", "Solid"};

    i32 current_item_index = static_cast<i32>(m_rasterizer_draw_type);
    if (ImGui::Combo("Rasterizer Draw Type", &current_item_index, draw_type_items.data(), draw_type_items.size()))
    {
        m_rasterizer_draw_type = static_cast<RasterizerDrawType>(current_item_index);
    }
}
#endif

void SkinnedModel::calculate_bounding_box()
{
    for (auto const& mesh : m_meshes)
        mesh->calculate_bounding_box();

    // TODO: Merge bounding boxes together
    if (!m_meshes.empty())
        bounds = m_meshes[0]->bounds;
}

void SkinnedModel::adjust_bounding_box()
{
    // TODO: If we merge bounding boxes together, I think we can just adjust the whole model bounding box
    for (auto const& mesh : m_meshes)
        mesh->adjust_bounding_box(entity->transform->get_model_matrix());

    if (!m_meshes.empty())
        bounds = m_meshes[0]->bounds;
}

BoundingBox SkinnedModel::get_adjusted_bounding_box(glm::mat4 const& model_matrix) const
{
    if (!m_meshes.empty())
        return m_meshes[0]->get_adjusted_bounding_box(model_matrix);

    return {};
}

bool SkinnedModel::is_skinned_model() const
{
    return true;
}

SkinnedModel::SkinnedModel(std::shared_ptr<Material> const& material) : Drawable(material)
{
}

void SkinnedModel::draw() const
{
    if (m_rasterizer_draw_type == RasterizerDrawType::None)
    {
        return;
    }

    // Either wireframe or solid for individual model
    Renderer::get_instance()->set_rasterizer_draw_type(m_rasterizer_draw_type);

    for (auto const& mesh : m_meshes)
        mesh->draw();

    Renderer::get_instance()->restore_default_rasterizer_draw_type();
}

void SkinnedModel::draw_instanced(i32 const size)
{
    for (auto const& mesh : m_meshes)
        mesh->draw_instanced(size);
}

std::shared_ptr<std::vector<glm::mat4>> SkinnedModel::get_skinning_matrices() const
{
    return Drawable::get_skinning_matrices();
}

void SkinnedModel::initialize()
{
    Drawable::initialize();
    AnimationEngine::get_instance()->register_skinned_model(std::dynamic_pointer_cast<SkinnedModel>(shared_from_this()));
}

void SkinnedModel::uninitialize()
{
    Drawable::uninitialize();
    AnimationEngine::get_instance()->unregister_skinned_model(std::dynamic_pointer_cast<SkinnedModel>(shared_from_this()));
}

void SkinnedModel::prepare()
{
    if (material->is_gpu_instanced)
    {
        if (material->first_drawable != nullptr)
            return;

        material->first_drawable = std::dynamic_pointer_cast<Drawable>(shared_from_this());
    }

    load_model(model_path, SkinningLoadMode::Rig);
    load_model(anim_path, SkinningLoadMode::Anim);
}

void SkinnedModel::reset()
{
    // m_meshes.clear();
    // m_loaded_textures.clear();
    animation = {};
    // m_scene = nullptr;
    m_bone_counter = 0;
    m_directory = "";
}

void SkinnedModel::reprepare()
{
    reset();
    prepare();
}

void SkinnedModel::load_model(std::string const& path, SkinningLoadMode const& load_mode)
{
    Assimp::Importer importer;
    m_scene = importer.ReadFile(path, aiProcess_PopulateArmatureData | aiProcess_Triangulate | aiProcess_FlipUVs);

    if ((m_scene == nullptr || m_scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || m_scene->mRootNode == nullptr)
        && load_mode == SkinningLoadMode::Rig)
    {
        std::cout << "Error. Failed loading a model: " << importer.GetErrorString() << "\n";
        return;
    }

    std::filesystem::path const filesystem_path = path;
    m_directory = filesystem_path.parent_path().string();

    if (entity != nullptr)
        m_cached_initial_model_pos = entity->transform->get_position();

    process_node(m_scene->mRootNode);
    initialize_animation();

    m_first_bone_calculation = true;
}

void SkinnedModel::process_node(aiNode const* node)
{
    for (u32 i = 0; i < node->mNumMeshes; ++i)
    {
        aiMesh const* mesh = m_scene->mMeshes[node->mMeshes[i]];
        m_meshes.emplace_back(proccess_mesh(mesh));
    }

    for (u32 i = 0; i < node->mNumChildren; ++i)
    {
        process_node(node->mChildren[i]);
    }
}

std::shared_ptr<Mesh> SkinnedModel::proccess_mesh(aiMesh const* mesh)
{
    std::vector<Vertex> vertices;
    std::vector<u32> indices;
    std::vector<std::shared_ptr<Texture>> textures;

    u32 vertex_id = 0;
    u32 bone_id = 0;
    float weight = 0.0f;

    for (u32 i = 0; i < mesh->mNumVertices; ++i)
    {
        Vertex vertex;
        set_vertex_bone_data_to_default(vertex);

        vertex.position = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);

        if (mesh->HasNormals())
        {
            vertex.normal = glm::vec3(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
        }

        if (mesh->mTextureCoords[0] != nullptr)
        {
            vertex.texture_coordinates = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
        }
        else
        {
            vertex.texture_coordinates = glm::vec2(0.0f, 0.0f);
        }

        vertices.push_back(vertex);
    }

    for (u32 i = 0; i < mesh->mNumFaces; ++i)
    {
        aiFace const face = mesh->mFaces[i];
        for (u32 k = 0; k < face.mNumIndices; k++)
        {
            indices.push_back(face.mIndices[k]);
        }
    }

    ////////////

    aiMaterial const* assimp_material = m_scene->mMaterials[mesh->mMaterialIndex];

    std::vector<std::shared_ptr<Texture>> diffuse_maps =
        load_material_textures(assimp_material, aiTextureType_DIFFUSE, TextureType::Diffuse);
    textures.insert(textures.end(), diffuse_maps.begin(), diffuse_maps.end());

    std::vector<std::shared_ptr<Texture>> specular_maps =
        load_material_textures(assimp_material, aiTextureType_SPECULAR, TextureType::Specular);
    textures.insert(textures.end(), specular_maps.begin(), specular_maps.end());

    extract_bone_weight_for_vertices(vertices, mesh);

    return ResourceManager::get_instance().load_mesh(m_meshes.size(), model_path, vertices, indices, textures, m_draw_type, material);
}

std::vector<std::shared_ptr<Texture>> SkinnedModel::load_material_textures(aiMaterial const* material, aiTextureType const type,
                                                                           TextureType const type_name)
{
    std::vector<std::shared_ptr<Texture>> textures;

    u32 const material_count = material->GetTextureCount(type);
    for (u32 i = 0; i < material_count; ++i)
    {
        aiString str;
        material->GetTexture(type, i, &str);

        bool is_already_loaded = false;
        for (auto const& loaded_texture : m_loaded_textures)
        {
            if (std::strcmp(loaded_texture->path.data(), str.C_Str()) == 0)
            {
                textures.push_back(loaded_texture);
                is_already_loaded = true;
                break;
            }
        }

        if (is_already_loaded)
            continue;

        auto file_path = std::string(str.C_Str());
        file_path = m_directory + '/' + file_path;

        TextureSettings settings = {};
        settings.flip_vertically = false;
        settings.filtering_min = TextureFiltering::Nearest;
        settings.filtering_max = TextureFiltering::Nearest;
        settings.filtering_mipmap = TextureFiltering::Nearest;

        std::shared_ptr<Texture> texture = ResourceManager::get_instance().load_texture(file_path, type_name, settings);
        textures.push_back(texture);
        m_loaded_textures.push_back(texture);
    }

    return textures;
}

void SkinnedModel::extract_bone_weight_for_vertices(std::vector<Vertex>& vertices, aiMesh const* mesh)
{
    for (int bone_index = 0; bone_index < mesh->mNumBones; ++bone_index)
    {
        int bone_id = -1;
        std::string bone_name = mesh->mBones[bone_index]->mName.C_Str();
        if (!m_bone_info_map.contains(bone_name))
        {
            BoneInfo new_bone_info;
            new_bone_info.id = m_bone_counter;
            new_bone_info.offset = AK::Math::ai_matrix_to_glm(mesh->mBones[bone_index]->mOffsetMatrix);
            m_bone_info_map[bone_name] = new_bone_info;
            bone_id = m_bone_counter;
            m_bone_counter++;
        }
        else
        {
            bone_id = m_bone_info_map[bone_name].id;
        }
        assert(bone_id != -1);
        auto const weights = mesh->mBones[bone_index]->mWeights;
        i32 const num_weights = mesh->mBones[bone_index]->mNumWeights;

        for (int weight_index = 0; weight_index < num_weights; ++weight_index)
        {
            i32 const vertex_id = weights[weight_index].mVertexId;
            float const weight = weights[weight_index].mWeight;
            assert(vertex_id <= vertices.size());
            set_vertex_bone_data(vertices[vertex_id], bone_id, weight);
        }
    }
}

void SkinnedModel::set_vertex_bone_data(Vertex& vertex, i32 bone_id, float weight)
{
    for (int i = 0; i < 4; ++i)
    {
        if (vertex.skin_indices[i] < 0 || vertex.skin_weights[i] < 0.000001f)
        {
            vertex.skin_weights[i] = weight;
            vertex.skin_indices[i] = bone_id;
            break;
        }
    }
}

void SkinnedModel::set_vertex_bone_data_to_default(Vertex& vertex)
{
    for (int i = 0; i < 4; i++)
    {
        vertex.skin_indices[i] = -1;
        vertex.skin_weights[i] = 0.0f;
    }
}

void SkinnedModel::calculate_bone_transform(AssimpNodeData const* node, glm::mat4 const& parent_transform)
{
    if (entity == nullptr)
        return;

    if (m_first_bone_calculation && glm::distance(m_cached_initial_model_pos, entity->transform->get_position()) > 0.0f)
    {
        m_cached_translated_model_pos = entity->transform->get_position();
        m_model_loading_offset = m_cached_translated_model_pos - m_cached_initial_model_pos;
        m_first_bone_calculation = false;
        entity->transform->set_position(entity->transform->get_position() - m_model_loading_offset);
    }

    std::string const node_name = node->name;
    glm::mat4 node_transform = node->transformation;

    if (m_skinning_matrices.empty())
        m_skinning_matrices.resize(SKINNING_BUFFER_SIZE);

    if (Bone* bone = find_bone(node_name))
    {
        glm::vec3 root_offset = {};
        glm::vec3 rotated_position = {};
        // Get the entity's current rotation in world/space (quaternion)
        glm::quat model_rotation = entity->transform->get_rotation();
        if (node_name == "root" && enable_root_motion && !m_blend_between_clips)
        {
            glm::vec3 scale = glm::vec3(0.0f);
            glm::vec3 skew = glm::vec3(0.0f);
            glm::vec4 perspective = glm::vec4(0.0f);
            glm::quat root_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

            decompose(bone->local_transform, scale, root_rotation, root_offset, skew, perspective);

            // Rotate the local position by the entity's model/world rotation
            rotated_position = model_rotation * (root_offset - animation.cached_root_offset);

            // Move root back to model (0,0,0)
            bone->local_transform = translate(glm::mat4(1.0f), glm::vec3(0.0f));
            // align_animation_to_vector(glm::vec3(0, 0, 1));

            // Offset entity ("should be capsule controller") by offset that root should traverse
            // Of course DO NOT do this when wrapping, because wrapping applies another offset, so they would be duplicated
            glm::vec3 final_position = entity->transform->get_position() + rotated_position;
            final_position.y = 0.0f;
            if (!animation.wrap_extracted_motion && !m_blend_between_clips)
                entity->transform->set_position(final_position);
            else
                animation.wrap_extracted_motion = false;

            // Cache last frame root offset
            animation.cached_root_offset = root_offset;

            // Wrap and offset model when reached the end of an animation
            if (animation.current_time > animation.duration)
            {
                animation.wrap_extracted_motion = true;
                animation.current_time = 0.0f;
            }
        }

        // For safety
        if (animation.current_time > animation.duration)
            animation.current_time = 0.0f;

        node_transform = bone->local_transform;

        // BTW zeby zrobic realignment rotacji mozna w jednej funkcji z sama poza, po prostu dodatkowa macierz z finalnej rotacji jako target blendu

        if (!m_blend_between_clips)
        {
            bone->update(animation.current_time);
        }
        else
        {
            // bone->blend_clips(blend_value, m_a_time, rotated_position);
            calculate_blending(*bone, blend_value, rotated_position);
        }
    }

    glm::mat4 const global_transformation = parent_transform * node_transform;

    auto bone_info_map = animation.bone_info_map;
    if (bone_info_map.contains(node_name))
    {
        i32 const index = bone_info_map[node_name].id;
        glm::mat4 const offset = bone_info_map[node_name].offset;
        m_skinning_matrices[index] = global_transformation * offset;
        animation.bones[index].model_transform = global_transformation * offset;
    }

    for (int i = 0; i < node->children_count; i++)
        calculate_bone_transform(&node->children[i], global_transformation);

    if (m_blend_between_clips && node_name == "root")
    {
        float new_blend_value = blend_value + delta_time * 0.5f;
        blend_value = std::clamp(new_blend_value, 0.0f, 1.0f);

        if (new_blend_value > 1.0f)
            stop_blending();
    }

    // Rotation realignment

    // align_animation_to_vector(m_b_alignment);

    // m_a_alignment.x = glm::radians(m_a_alignment.x);
    // m_a_alignment.y = glm::radians(m_a_alignment.y);
    // m_a_alignment.z = glm::radians(m_a_alignment.z);
    // glm::quat a = glm::quat(m_a_alignment);
    //
    // m_b_alignment.x = glm::radians(m_b_alignment.x);
    // m_b_alignment.y = glm::radians(m_b_alignment.y);
    // m_b_alignment.z = glm::radians(m_b_alignment.z);
    // glm::quat b = glm::quat(m_b_alignment);
    //
    // a = glm::normalize(a);
    // b = glm::normalize(b);
    //
    // glm::quat alignment = glm::slerp(a, b, blend_value);
    // alignment = glm::normalize(alignment);
    //
    // glm::vec3 euler_radians = glm::eulerAngles(alignment);
    // glm::vec3 euler = {0.0f, glm::degrees(euler_radians.y), 0.0f};

    // if (node_name == "root")
    // {

    // auto const v = glm::normalize(glm::mix(m_a_alignment, m_b_alignment, blend_value));
    // align_animation_to_vector(v);
    Debug::log("Current time: " + std::to_string(animation.current_time) + ", b_time: " + std::to_string(m_b_time));
    // }
}

void SkinnedModel::align_animation_to_vector(glm::vec3 const& v)
{
    entity->transform->set_euler_angles(calculate_animation_alignment(v));
}

glm::vec3 SkinnedModel::calculate_animation_alignment(glm::vec3 const& v)
{
    glm::vec3 result = {};
    if (Bone* bone = find_bone("Hips"))
    {
        glm::vec3 scale = glm::vec3(0.0f);
        glm::vec3 pos = glm::vec3(0.0f);
        glm::vec3 skew = glm::vec3(0.0f);
        glm::vec4 perspective = glm::vec4(0.0f);
        glm::quat q_h = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

        decompose(bone->local_transform, scale, q_h, pos, skew, perspective);

        auto facing_direction = q_h * glm::vec3(0.0f, 0.0f, -1.0f); // Z vector is forward, but negative for some reason (trial & error)
        facing_direction.y = 0.0f;
        facing_direction = normalize(facing_direction);

        float dot = glm::dot(facing_direction, v);
        glm::vec3 cross = glm::cross(facing_direction, v);
        float angle = glm::acos(dot);
        float angle_deg = glm::degrees(angle);

        if (cross.y > 0)
            angle_deg = 360.0f - angle_deg;

        result = {0.0f, -angle_deg, 0.0f};
    }
    return result;
}

bool SkinnedModel::get_update_in_anim_engine() const
{
    return m_update_in_anim_engine;
}

void SkinnedModel::reset_anim_data_only()
{
    animation = {};
    m_bone_counter = 0;
    m_directory = "";
}

void SkinnedModel::calculate_blending(Bone& bone, float alpha, glm::vec3 const& blend_offset)
{
    glm::mat4 translation = glm::mat4(1.0f); // interpolate_position(alpha);
    glm::mat4 rotation = glm::mat4(1.0f); // interpolate_rotation(alpha);

    Bone* bone_a = nullptr;
    Bone* bone_b = nullptr;

    {
        auto const iter = std::ranges::find_if(animation_a.bones, [&](Bone const& b) { return b.name == bone.name; });
        bone_a = &(*iter);
    }

    {
        auto const iter = std::ranges::find_if(animation_b.bones, [&](Bone const& b) { return b.name == bone.name; });
        bone_b = &(*iter);
    }

    // Interpolate position
    if (bone.name == "root")
    {
        translation = glm::translate(glm::mat4(1.0f), -blend_offset);
    }
    else
    {
        auto const p0_index = bone_a->get_position_index(m_a_time);
        auto const p1_index = bone_b->get_position_index(m_b_time);
        glm::vec3 const final_position = glm::mix(bone_a->positions[p0_index].position, bone_b->positions[p1_index].position, alpha);
        translation = glm::translate(glm::mat4(1.0f), final_position);
    }

    {
        auto const p0_index = bone_a->get_rotation_index(m_a_time);
        auto const p1_index = bone_b->get_rotation_index(m_b_time);
        glm::quat final_rotation = glm::slerp(bone_a->rotations[p0_index].orientation, bone_b->rotations[p1_index].orientation, alpha);
        final_rotation = glm::normalize(final_rotation);
        rotation = glm::toMat4(final_rotation);
    }

    bone.local_transform = translation * rotation * glm::mat4(1.0f); // skalujesz zalujesz
}

void SkinnedModel::start_blending(std::string const& blend_to_path, float b_time, glm::vec3 const& facing_a, glm::vec3 const& facing_b)
{
    m_update_in_anim_engine = false;
    blend_value = 0.0f;
    animation_a = animation;
    m_a_time = animation.current_time;
    m_a_alignment = facing_a;
    m_b_alignment = facing_b;

    reset();

    anim_path = blend_to_path;
    load_model(anim_path, SkinningLoadMode::Anim);

    m_b_time = b_time;
    animation.current_time = b_time;

    animation_b = animation;
    m_blend_between_clips = true;
    m_update_in_anim_engine = true;
}

void SkinnedModel::stop_blending()
{
    m_blend_between_clips = false;
    animation = animation_b;
}

void SkinnedModel::initialize_animation()
{
    Assimp::Importer importer;
    aiScene const* scene = importer.ReadFile(anim_path, aiProcess_Triangulate);
    assert(scene && scene->mRootNode);
    auto const assimp_animation = scene->mAnimations[0];
    animation.duration = assimp_animation->mDuration;
    animation.ticks_per_second = assimp_animation->mTicksPerSecond;
    read_hierarchy_data(animation.root_node, scene->mRootNode);
    read_missing_bones(assimp_animation);
}

void SkinnedModel::read_hierarchy_data(AssimpNodeData& dest, aiNode const* src)
{
    assert(src);

    dest.name = src->mName.data;
    dest.transformation = AK::Math::ai_matrix_to_glm(src->mTransformation);
    dest.children_count = src->mNumChildren;

    for (int i = 0; i < src->mNumChildren; i++)
    {
        AssimpNodeData newData;
        read_hierarchy_data(newData, src->mChildren[i]);
        dest.children.push_back(newData);
    }
}

void SkinnedModel::read_missing_bones(aiAnimation const* assimp_animation)
{
    u32 const size = assimp_animation->mNumChannels;

    std::map<std::string, BoneInfo> new_bone_info_map = m_bone_info_map;
    u32 bone_count = m_bone_counter;

    for (int i = 0; i < size; i++)
    {
        auto const channel = assimp_animation->mChannels[i];
        std::string boneName = channel->mNodeName.data;

        if (!new_bone_info_map.contains(boneName))
        {
            new_bone_info_map[boneName].id = bone_count;
            bone_count++;
        }

        Bone bone;
        bone.init(channel->mNodeName.data, new_bone_info_map[channel->mNodeName.data].id, channel);
        animation.bones.push_back(bone);
    }

    animation.bone_info_map = new_bone_info_map;
}

Bone* SkinnedModel::find_bone(std::string const& name)
{
    auto const iter = std::ranges::find_if(animation.bones, [&](Bone const& bone) { return bone.name == name; });
    if (iter == animation.bones.end())
        return nullptr;

    return &(*iter);
}
