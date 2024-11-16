#pragma once
#include <string>
#include <vector>

#include <assimp/material.h>

#include "AK/Badge.h"
#include "Mesh.h"
#include "Rig.h"
#include "Texture.h"

#include <map>

struct BoneInfo;
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

    virtual std::shared_ptr<std::vector<glm::mat4>> get_skinning_matrices() const override;

    virtual void initialize() override;
    virtual void uninitialize() override;

    virtual void prepare();
    virtual void reset();
    virtual void reprepare() override;

    virtual void calculate_bounding_box() override;
    virtual void adjust_bounding_box() override;
    virtual BoundingBox get_adjusted_bounding_box(glm::mat4 const& model_matrix) const override;

    virtual bool is_skinned_model() const override;
    void calculate_bone_transform(AssimpNodeData const* node, glm::mat4 const& parent_transform);

    std::string model_path = "./res/models/enemy/enemy.gltf";
    std::string anim_path = "./res/anims/conv77_12_Anim.gltf";

    bool enable_root_motion = true;

    NON_SERIALIZED
    Animation animation = {};

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
    // void extract_bone_data(aiNode const* node, SkinningLoadMode mode);
    // void extract_bone_data_from_mesh(aiMesh const* mesh, SkinningLoadMode mode);

    void extract_bone_weight_for_vertices(std::vector<Vertex>& vertices, aiMesh const* mesh);
    void set_vertex_bone_data(Vertex& vertex, i32 bone_id, float weight);
    void set_vertex_bone_data_to_default(Vertex& vertex);

    void initialize_animation();
    void read_hierarchy_data(AssimpNodeData& dest, aiNode const* src);
    void read_missing_bones(aiAnimation const* assimp_animation);
    Bone* find_bone(std::string const& name);

    aiScene const* m_scene = nullptr;
    std::map<std::string, BoneInfo> m_bone_info_map = {};
    u32 m_bone_counter = 0;

    std::string m_directory = "";
    std::vector<std::shared_ptr<Texture>> m_loaded_textures = {};
};
