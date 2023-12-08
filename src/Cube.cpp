#include "Cube.h"

#include "Globals.h"

std::shared_ptr<Cube> Cube::create(std::shared_ptr<MaterialInstance> const& material_instance)
{
    auto cube = std::make_shared<Cube>(material_instance);
    cube->prepare();

    return cube;
}

std::shared_ptr<Cube> Cube::create(std::string const& diffuse_texture_path, std::shared_ptr<MaterialInstance> const& material_instance)
{
    auto cube = std::make_shared<Cube>(diffuse_texture_path, material_instance);
    cube->prepare();

    return cube;
}

std::shared_ptr<Cube> Cube::create(std::string const& diffuse_texture_path, std::string const& specular_texture_path,
                                   std::shared_ptr<MaterialInstance> const& material_instance)
{
    auto cube = std::make_shared<Cube>(diffuse_texture_path, specular_texture_path, material_instance);
    cube->prepare();

    return cube;
}

Cube::Cube(std::shared_ptr<MaterialInstance> const& material_instance) : Model(material_instance)
{
    draw_type = GL_TRIANGLES;
}

Cube::Cube(std::string diffuse_texture_path, std::shared_ptr<MaterialInstance> const& material_instance)
    : Model(material_instance), diffuse_texture_path(std::move(diffuse_texture_path))
{
    draw_type = GL_TRIANGLES;
}

Cube::Cube(std::string diffuse_texture_path, std::string specular_texture_path, std::shared_ptr<MaterialInstance> const& material_instance)
    : Model(material_instance), diffuse_texture_path(std::move(diffuse_texture_path)), specular_texture_path(
          std::move(specular_texture_path))
{
    draw_type = GL_TRIANGLES;
}

void Cube::prepare()
{
    if (material_instance->is_gpu_instanced)
    {
        if (material_instance->first_drawable != nullptr)
            return;

        material_instance->first_drawable = std::dynamic_pointer_cast<Drawable>(shared_from_this());
    }

    meshes.emplace_back(create_cube());
}

void Cube::reset()
{
    meshes.clear();
}

void Cube::reprepare()
{
    Cube::reset();
    Cube::prepare();
}

Mesh Cube::create_cube() const
{
    std::vector<Vertex> const vertices = InternalMeshData::cube.vertices;
    std::vector<uint32_t> const indices = InternalMeshData::cube.indices;
    std::vector<Texture> textures;

    std::vector<Texture> diffuse_maps = {};
    if (!diffuse_texture_path.empty())
        diffuse_maps.emplace_back(load_texture(diffuse_texture_path, "texture_diffuse"));

    std::vector<Texture> specular_maps = {};
    if (!specular_texture_path.empty())
        specular_maps.emplace_back(load_texture(specular_texture_path, "texture_specular"));

    textures.insert(textures.end(), diffuse_maps.begin(), diffuse_maps.end());
    textures.insert(textures.end(), specular_maps.begin(), specular_maps.end());

    return Mesh::create(vertices, indices, textures, draw_type, material_instance);
}

Texture Cube::load_texture(std::string const& path, std::string const& type) const
{
    Texture texture;
    texture.id = texture_from_file(path.c_str());
    texture.type = type;
    texture.path = diffuse_texture_path;
    return texture;
}
