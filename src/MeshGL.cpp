#include "MeshGL.h"

#include <iostream>
#include <glad/glad.h>

#include "Globals.h"
#include "Texture.h"

MeshGL::MeshGL(AK::Badge<MeshFactory>, std::vector<Vertex> const& vertices, std::vector<std::uint32_t> const& indices,
               std::vector<Texture> const& textures, DrawType const draw_type, std::shared_ptr<Material> const& material,
               DrawFunctionType const draw_function):
    Mesh(vertices, indices, textures, draw_type, material, draw_function)
{
    switch (draw_type)
    {
    case DrawType::Triangles:
        m_draw_typeGL = GL_TRIANGLES;
        break;
    case DrawType::TriangleStrip:
        m_draw_typeGL = GL_TRIANGLE_STRIP;
        break;
    case DrawType::Patches:
        m_draw_typeGL = GL_PATCHES;
        break;
    case DrawType::TriangleFan:
        m_draw_typeGL = GL_TRIANGLE_FAN;
        break;
    case DrawType::LineLoop:
        m_draw_typeGL = GL_LINE_LOOP;
        break;
    case DrawType::LineStrip:
        m_draw_typeGL = GL_LINE_STRIP;
        break;
    case DrawType::Lines:
        m_draw_typeGL = GL_LINES;
        break;
    case DrawType::Points:
        m_draw_typeGL = GL_POINTS;
        break;
    default:
        std::unreachable();
    }

    glGenVertexArrays(1, &m_VAO);
    glGenBuffers(1, &m_VBO);
    glGenBuffers(1, &m_EBO);

    glBindVertexArray(m_VAO);

    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO);
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

    if (draw_type == DrawType::Patches)
    {
        glPatchParameteri(GL_PATCH_VERTICES, 4); // FIXME: Hardcoded patch size
    }

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

MeshGL::MeshGL(MeshGL&& mesh) noexcept : Mesh(mesh.vertices, mesh.indices, mesh.textures, mesh.m_draw_type, mesh.material, mesh.m_draw_function)
{
    m_VAO = mesh.m_VAO;
    m_VBO = mesh.m_VBO;
    m_EBO = mesh.m_EBO;

    mesh.m_VAO = 0;
    mesh.m_VBO = 0;
    mesh.m_EBO = 0;

    mesh.vertices.clear();
    mesh.indices.clear();
    mesh.textures.clear();
}

MeshGL::~MeshGL()
{
    for (auto const& texture : textures)
    {
        glDeleteTextures(1, &texture.id);
    }

    vertices.clear();
    indices.clear();
    textures.clear();

    glDeleteBuffers(1, &m_EBO);
    glDeleteBuffers(1, &m_VBO);
    glDeleteVertexArrays(1, &m_VAO);
}

void MeshGL::draw() const
{
    bind_textures();

    // Draw mesh
    glBindVertexArray(m_VAO);

    if (m_draw_function == DrawFunctionType::NotIndexed)
        glDrawArrays(m_draw_typeGL, 0, static_cast<int>(vertices.size()));
    else
        glDrawElements(m_draw_typeGL, static_cast<int>(indices.size()), GL_UNSIGNED_INT, 0);

    glBindVertexArray(0);

    unbind_textures();
}

void MeshGL::draw(uint32_t const size, void const* offset) const
{
    bind_textures();

    glBindVertexArray(m_VAO);

    if (m_draw_function == DrawFunctionType::Indexed)
    {
        glDrawElements(m_draw_typeGL, size, GL_UNSIGNED_INT, offset);
    }
    else
    {
        std::cout << "Non indexed drawing with offset is not currently supported." << "\n";
    }

    glBindVertexArray(0);

    unbind_textures();
}

void MeshGL::draw_instanced(int32_t const size) const
{
    bind_textures();

    glBindVertexArray(m_VAO);
    glDrawElementsInstanced(
        GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, (void*)0, size
    );

    unbind_textures();
}

void MeshGL::bind_textures() const
{
    std::uint32_t diffuse_number = 1;
    std::uint32_t specular_number = 1;
    std::uint32_t height_number = 1;

    for (std::uint32_t i = 0; i < textures.size(); ++i)
    {
        glActiveTexture(GL_TEXTURE0 + i);

        std::string number;
        std::string name = "material.";

        if (textures[i].type == TextureType::Diffuse)
        {
            name += "texture_diffuse";
            number = std::to_string(diffuse_number++);
        }
        else if (textures[i].type == TextureType::Specular)
        {
            name += "texture_specular";
            number = std::to_string(specular_number++);
        }
        else if (textures[i].type == TextureType::Heightmap)
        {
            name += "texture_height";
            number = std::to_string(height_number++);
        }

        material->shader->set_int(name + number, i);
        glBindTexture(GL_TEXTURE_2D, textures[i].id);
    }

    if (textures.empty())
    {
        glActiveTexture(GL_TEXTURE0);

        material->shader->set_int("material.texture_diffuse1", 0);

        glBindTexture(GL_TEXTURE_2D, InternalMeshData::white_texture.id);
    }
}

void MeshGL::unbind_textures() const
{
    glActiveTexture(GL_TEXTURE0);

    glBindTexture(GL_TEXTURE_2D, 0);
}
