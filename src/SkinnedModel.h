#pragma once
#include <string>
#include <vector>

#include <assimp/material.h>

#include "AK/Badge.h"
#include "Mesh.h"
#include "Rig.h"
#include "Texture.h"

#include <unordered_map>

struct aiBone;
struct aiMaterial;
struct aiMesh;
struct aiScene;
struct aiNode;

enum class SkinningLoadMode
{
    Rig,
    Anim
};

class SkinnedModel : public Drawable
{
public:
    static std::shared_ptr<SkinnedModel> create();
    static std::shared_ptr<SkinnedModel> create(std::string const& model_path, std::string const& anim_path,
                                                std::shared_ptr<Material> const& material);
    static std::shared_ptr<SkinnedModel> create(std::shared_ptr<Mesh> mesh, std::shared_ptr<Material> const& material);
    static std::shared_ptr<SkinnedModel> create(std::shared_ptr<Material> const& material);

    explicit SkinnedModel(AK::Badge<SkinnedModel>, std::string const& model_path, std::string const& anim_path,
                          std::shared_ptr<Material> const& material);
    explicit SkinnedModel(AK::Badge<SkinnedModel>, std::shared_ptr<Material> const& material);

#if EDITOR
    virtual void draw_editor() override;
#endif

    virtual void draw() const override;
    virtual void draw_instanced(i32 const size) override;

    glm::mat4 const* get_skinning_matrices() const;

    virtual void initialize() override;
    virtual void uninitialize() override;

    virtual void prepare();
    virtual void reset();
    virtual void reprepare() override;

    virtual void calculate_bounding_box() override;
    virtual void adjust_bounding_box() override;
    virtual BoundingBox get_adjusted_bounding_box(glm::mat4 const& model_matrix) const override;

    virtual bool is_skinned_model() const override;

    std::string model_path = "./res/models/enemy/enemy.gltf";
    std::string anim_path = "./res/models/enemy/enemy.gltf";

    NON_SERIALIZED
    std::vector<glm::mat4> skinning_matrices = {};

protected:
    explicit SkinnedModel(std::shared_ptr<Material> const& material);

    DrawType m_draw_type = DrawType::Triangles;
    std::vector<std::shared_ptr<Mesh>> m_meshes = {};

private:
    void load_model(std::string const& path, SkinningLoadMode const& load_mode);
    void process_node(aiNode const* node);
    std::shared_ptr<Mesh> proccess_mesh(aiMesh const* mesh);
    std::vector<std::shared_ptr<Texture>> load_material_textures(aiMaterial const* material, aiTextureType type,
                                                                 TextureType const type_name);
    void extract_bone_data(aiNode const* node, SkinningLoadMode mode);
    void extract_bone_data_from_mesh(aiMesh const* mesh, SkinningLoadMode mode);

    aiScene const* m_scene = nullptr;
    std::unordered_map<std::string, i32> bone_names_to_ids = {};
    std::unordered_map<i32, std::string> bone_ids_to_names = {};

    std::vector<AK::xform> local_pose = {};
    std::vector<AK::xform> model_pose = {};

    std::string m_directory = "";
    std::vector<std::shared_ptr<Texture>> m_loaded_textures = {};

    Rig m_rig = {};
};
