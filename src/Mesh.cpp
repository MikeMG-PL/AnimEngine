#include "Mesh.h"

#include <iostream>
#include <string>
#include <utility>
#include <glad/glad.h>

#include "Camera.h"
#include "Shader.h"
#include "Texture.h"
#include "Vertex.h"

Mesh::Mesh(std::vector<Vertex> vertices, std::vector<std::uint32_t> indices, std::vector<Texture> textures, GLenum draw_type, std::shared_ptr<Material> const& material)
    : Drawable(material), vertices(std::move(vertices)), indices(std::move(indices)), textures(std::move(textures)), draw_type(draw_type)
{
}

void Mesh::setup_mesh()
{
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(std::uint32_t), indices.data(), GL_STATIC_DRAW);

    // FIXME: Not all shaders have all these attributes

    // Vertex positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);

    // Vertex normals
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    // Vertex texture coordinates
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texture_coordinates));

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

Mesh::~Mesh()
{
    for (auto const& texture : textures)
    {
        glDeleteTextures(1, &texture.id);
    }

    vertices.clear();
    indices.clear();
    textures.clear();

    glDeleteBuffers(1, &EBO);
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &VAO);
}

Mesh::Mesh(Mesh&& mesh) noexcept : Mesh(mesh.vertices, mesh.indices, mesh.textures, mesh.draw_type, mesh.material)
{
    VAO = mesh.VAO;
    VBO = mesh.VBO;
    EBO = mesh.EBO;

    mesh.VAO = 0;
    mesh.VBO = 0;
    mesh.EBO = 0;

    mesh.vertices.clear();
    mesh.indices.clear();
    mesh.textures.clear();
}

Mesh Mesh::create(std::vector<Vertex> const& vertices, std::vector<std::uint32_t> const& indices, std::vector<Texture> const& textures,
                  GLenum const draw_type, std::shared_ptr<Material> const& material)
{
    auto mesh = Mesh(vertices, indices, textures, draw_type, material);
    mesh.setup_mesh();
    return mesh;
}

void Mesh::draw() const
{
    // Bind textures
    std::uint32_t diffuse_number = 1;
    std::uint32_t specular_number = 1;

    for (std::uint32_t i = 0; i < textures.size(); ++i)
    {
        glActiveTexture(GL_TEXTURE0 + i);

        std::string number;
        std::string name = textures[i].type;
        if (name == "texture_diffuse")
            number = std::to_string(diffuse_number++);
        else if (name == "texture_specular")
            number = std::to_string(specular_number++);

        material->shader->set_int(name + number, i);
        glBindTexture(GL_TEXTURE_2D, textures[i].id);
    }

    // Draw mesh
    glBindVertexArray(VAO);
    if (draw_type == GL_LINE_LOOP)
        glDrawArrays(draw_type, 0, static_cast<int>(vertices.size()));
    else
        glDrawElements(draw_type, static_cast<int>(indices.size()), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    glActiveTexture(GL_TEXTURE0);

    glBindTexture(GL_TEXTURE_2D, 0);
}
