#include "Cube.h"

#include <memory>

#include "Globals.h"
#include "MeshFactory.h"

std::shared_ptr<Cube> Cube::create(std::shared_ptr<Material> const& material, bool const big_cube)
{
    auto cube = std::make_shared<Cube>(AK::Badge<Cube> {}, material);
    cube->big_cube = big_cube;
    cube->prepare();

    return cube;
}

std::shared_ptr<Cube> Cube::create(std::string const& diffuse_texture_path, std::shared_ptr<Material> const& material, bool const big_cube)
{
    AK::Badge<Cube> badge;
    auto cube = std::make_shared<Cube>(AK::Badge<Cube> {}, diffuse_texture_path, material);
    cube->big_cube = big_cube;
    cube->prepare();

    return cube;
}

std::shared_ptr<Cube> Cube::create(std::string const& diffuse_texture_path, std::string const& specular_texture_path,
                                   std::shared_ptr<Material> const& material, bool const big_cube)
{
    AK::Badge<Cube> badge;
    auto cube = std::make_shared<Cube>(AK::Badge<Cube> {}, diffuse_texture_path, specular_texture_path, material);
    cube->big_cube = big_cube;
    cube->prepare();

    return cube;
}

Cube::Cube(AK::Badge<Cube>, std::shared_ptr<Material> const& material) : Model(material)
{
    draw_type = DrawType::Triangles;
}

Cube::Cube(AK::Badge<Cube>, std::string const& diffuse_texture_path, std::shared_ptr<Material> const& material)
    : Model(material), diffuse_texture_path(diffuse_texture_path)
{
    draw_type = DrawType::Triangles;
}

Cube::Cube(AK::Badge<Cube>, std::string const& diffuse_texture_path, std::string const& specular_texture_path, std::shared_ptr<Material> const& material)
    : Model(material), diffuse_texture_path(diffuse_texture_path), specular_texture_path(specular_texture_path)
{
    draw_type = DrawType::Triangles;
}

std::string Cube::get_name() const
{
    std::string const name = typeid(decltype(*this)).name();
    return name.substr(6);
}

void Cube::prepare()
{
    if (material->is_gpu_instanced)
    {
        if (material->first_drawable != nullptr)
            return;

        material->first_drawable = std::dynamic_pointer_cast<Drawable>(shared_from_this());
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

std::shared_ptr<Mesh> Cube::create_cube() const
{
    std::vector<Vertex> const vertices = big_cube ? InternalMeshData::big_cube.vertices : InternalMeshData::cube.vertices;
    std::vector<uint32_t> const indices = big_cube ? InternalMeshData::big_cube.indices : InternalMeshData::cube.indices;
    std::vector<Texture> textures;

    std::vector<Texture> diffuse_maps = {};
    if (!diffuse_texture_path.empty())
        diffuse_maps.emplace_back(load_texture(diffuse_texture_path, "texture_diffuse"));

    std::vector<Texture> specular_maps = {};
    if (!specular_texture_path.empty())
        specular_maps.emplace_back(load_texture(specular_texture_path, "texture_specular"));

    textures.insert(textures.end(), diffuse_maps.begin(), diffuse_maps.end());
    textures.insert(textures.end(), specular_maps.begin(), specular_maps.end());

    return MeshFactory::create(vertices, indices, textures, draw_type, material);
}
