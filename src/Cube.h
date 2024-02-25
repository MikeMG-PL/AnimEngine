#pragma once

#include "Model.h"
#include "AK/Badge.h"

class Cube final : public Model
{
public:
    static std::shared_ptr<Cube> create(std::shared_ptr<Material> const& material, bool const big_cube = false);
    static std::shared_ptr<Cube> create(std::string const& diffuse_texture_path, std::shared_ptr<Material> const& material, bool const big_cube = false);
    static std::shared_ptr<Cube> create(std::string const& diffuse_texture_path, std::string const& specular_texture_path, std::shared_ptr<Material> const& material, bool const big_cube = false);

    explicit Cube(AK::Badge<Cube>, std::shared_ptr<Material> const& material);
    explicit Cube(AK::Badge<Cube>, std::string const& diffuse_texture_path, std::shared_ptr<Material> const& material);
    explicit Cube(AK::Badge<Cube>, std::string const& diffuse_texture_path, std::string const& specular_texture_path, std::shared_ptr<Material> const& material);

    std::string get_name() const override;

    virtual void prepare() override;
    virtual void reset() override;
    virtual void reprepare() override;

private:
    [[nodiscard]] std::shared_ptr<Mesh> create_cube() const;

    std::string diffuse_texture_path;
    std::string specular_texture_path;

    bool big_cube = false;

    friend class SceneSerializer;
};
