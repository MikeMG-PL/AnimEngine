#include "SkinnedModel.h"

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

glm::mat4 const* SkinnedModel::get_skinning_matrices() const
{
    return skinning_matrices.data();
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
    m_meshes.clear();
    m_loaded_textures.clear();
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
    if (load_mode == SkinningLoadMode::Anim)
    {
        if (!m_scene)
        // if (!m_scene || !m_scene->HasAnimations())
        {
            std::cout << "ERROR::ASSIMP::" << importer.GetErrorString() << "::LOADMODE::" << std::to_string(static_cast<int>(load_mode))
                      << std::endl;
            return;
        }
    }

    std::filesystem::path const filesystem_path = path;
    m_directory = filesystem_path.parent_path().string();

    if (load_mode == SkinningLoadMode::Rig)
    {
        // Extract rig data from the scene
        extract_bone_data(m_scene->mRootNode, load_mode);

        // Process meshes and use the extracted bone data
        process_node(m_scene->mRootNode);
    }
    if (load_mode == SkinningLoadMode::Anim)
    {
        for (u32 i = 0; i < m_rig.num_bones; i++)
        {
            AK::xform transform;
            local_pose.emplace_back(transform);
        }

        process_node(m_scene->mRootNode);
    }
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

    glm::ivec4 skinIndices = {-1, -1, -1, -1};
    glm::vec4 skinWeights = {0.0f, 0.0f, 0.0f, 0.0f};

    std::map<u32, glm::ivec4> vertex_to_bone_indices;
    std::map<u32, glm::vec4> vertex_to_bone_weights;
    Vertex vertex;
    u32 vertex_id = 0;
    u32 bone_id = 0;
    float weight = 0.0f;

    for (int i = 0; i < mesh->mNumBones; i++)
    {
        for (int j = 0; j < mesh->mBones[i]->mNumWeights; j++)
        {
            vertex_id = mesh->mBones[i]->mWeights[j].mVertexId;
            bone_id = i;
            weight = mesh->mBones[i]->mWeights[j].mWeight;

            // Skip zero weights
            if (weight < 1e-6) // A small threshold to ignore near-zero weights
                continue;

            // Initialize the ivec4 with -1s and vec4 with 0s - default values
            if (!vertex_to_bone_indices.contains(vertex_id))
            {
                vertex_to_bone_indices.try_emplace(vertex_id, skinIndices);
                vertex_to_bone_weights.try_emplace(vertex_id, skinWeights);
            }

            // Assign bone influence if an empty slot is found
            for (int b = 0; b < 4; ++b)
            {
                if (vertex_to_bone_indices[vertex_id][b] == -1)
                {
                    vertex_to_bone_indices[vertex_id][b] = bone_id;
                    vertex_to_bone_weights[vertex_id][b] = weight;
                    break; // Exit the loop once the assignment is done
                }
            }
        }
    }

    for (u32 i = 0; i < mesh->mNumVertices; ++i)
    {
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

        vertex.skin_indices = vertex_to_bone_indices[i];
        vertex.skin_weights = vertex_to_bone_weights[i];

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

    if (!local_pose.empty())
    {
        // Fill model_pose with local_pose matrices to prevent from being empty
        model_pose = local_pose;

        // Local Space --> Model Space
        m_rig.local_to_model(local_pose, model_pose);

        skinning_matrices.clear();

        for (u16 i = 0; i < SKINNING_BUFFER_SIZE; i++)
        {
            skinning_matrices.emplace_back(1.0f);
        }

        for (u32 i = 0; i < m_rig.num_bones; i++)
        {
            skinning_matrices[i] = AK::Math::xform_to_mat4(model_pose[i]);
        }

        std::shared_ptr<Drawable> this_drawable = std::static_pointer_cast<Drawable>(shared_from_this());
        RendererDX11::get_instance_dx11()->set_skinning_buffer(this_drawable, skinning_matrices.data());
    }

    ////////////

    aiMaterial const* assimp_material = m_scene->mMaterials[mesh->mMaterialIndex];

    std::vector<std::shared_ptr<Texture>> diffuse_maps =
        load_material_textures(assimp_material, aiTextureType_DIFFUSE, TextureType::Diffuse);
    textures.insert(textures.end(), diffuse_maps.begin(), diffuse_maps.end());

    std::vector<std::shared_ptr<Texture>> specular_maps =
        load_material_textures(assimp_material, aiTextureType_SPECULAR, TextureType::Specular);
    textures.insert(textures.end(), specular_maps.begin(), specular_maps.end());

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

void SkinnedModel::extract_bone_data(aiNode const* node, SkinningLoadMode mode)
{
    // Process each mesh node if it contains bones
    if (node->mNumMeshes > 0)
    {
        aiMesh const* mesh = m_scene->mMeshes[node->mMeshes[0]]; // Assuming one mesh per node for simplicity
        extract_bone_data_from_mesh(mesh, mode);
    }

    // Recursively process child nodes
    for (unsigned int i = 0; i < node->mNumChildren; i++)
    {
        extract_bone_data(node->mChildren[i], mode);
    }
}

void SkinnedModel::extract_bone_data_from_mesh(aiMesh const* mesh, SkinningLoadMode mode)
{
    // Extract bone data from the mesh
    for (unsigned int i = 0; i < mesh->mNumBones; i++)
    {
        // Extract bone name, parent index, and bind pose
        aiBone const* bone = mesh->mBones[i];
        std::string parent_bone_name;
        std::string bone_name = bone->mName.C_Str();

        if (bone->mNode == nullptr || bone->mNode->mParent == nullptr)
            parent_bone_name = "";
        else
            parent_bone_name = bone->mNode->mParent->mName.C_Str();

        // Find parent bone in the mBones array
        i32 parentIndex = -1;

        for (u32 j = 0; j < mesh->mNumBones; j++)
        {
            std::string const checkedBoneName = mesh->mBones[j]->mName.C_Str();
            if (i != j && parent_bone_name == checkedBoneName)
            {
                // Found the parent bone
                parentIndex = j;
                break;
            }
        }

        aiMatrix4x4 const ai_inverse_bind_pose = bone->mOffsetMatrix;
        aiQuaternion q = {1.0f, 0.0f, 0.0f, 0.0f};
        aiVector3D v = {0.0f, 0.0f, 0.0f};
        ai_inverse_bind_pose.DecomposeNoScaling(q, v);
        AK::xform ref_pose = {{v.x, v.y, v.z}, {q.w, q.x, q.y, q.z}};

        m_rig.bone_names.push_back(bone_name);
        m_rig.parents.push_back(parentIndex);
        m_rig.ref_pose.push_back(ref_pose);
    }
    m_rig.num_bones = mesh->mNumBones;
}
