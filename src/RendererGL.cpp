#include "RendererGL.h"

#include <array>
#include <format>
#include <iostream>

#include "Camera.h"
#include "Entity.h"

std::shared_ptr<RendererGL> RendererGL::create()
{
    auto renderer = std::make_shared<RendererGL>(AK::Badge<RendererGL> {});

    assert(instance == nullptr);

    set_instance(renderer);

    return renderer;
}

RendererGL::RendererGL(AK::Badge<RendererGL>)
{
}

void RendererGL::initialize_global_renderer_settings()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_MULTISAMPLE);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void RendererGL::initialize_buffers(size_t const max_size)
{
    glGenBuffers(1, &gpu_instancing_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, gpu_instancing_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, max_size * sizeof(glm::mat4), nullptr, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, gpu_instancing_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    glGenBuffers(1, &bounding_boxes_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, bounding_boxes_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, max_size * sizeof(BoundingBoxShader), nullptr, GL_DYNAMIC_READ);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, bounding_boxes_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    glGenBuffers(1, &visible_instances_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, visible_instances_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, max_size * sizeof(GLuint), nullptr, GL_DYNAMIC_READ);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, visible_instances_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void RendererGL::perform_frustum_culling(std::shared_ptr<Material> const& material) const
{
    frustum_culling_shader->use();

    // Set frustum planes
    auto const frustum_planes = Camera::get_main_camera()->get_frustum_planes();

    for (uint32_t i = 0; i < 6; ++i)
    {
        frustum_culling_shader->set_vec4(std::format("frustumPlanes[{}]", i), frustum_planes[i]);
    }

    // Send bounding boxes
    // TODO: Batch them with all other existing objects and send only once
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, bounding_boxes_ssbo);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, material->bounding_boxes.size() * sizeof(BoundingBoxShader), material->bounding_boxes.data());
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    // Run frustum culling
    glDispatchCompute((material->drawables.size() / 1024) + 1, 1, 1);

    // Wait for SSBO write to complete
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // Read visible_instances SSBO which has value of 1 when a corresponding object is visible and 0 if it is not visible
    auto* visible_instances = new GLuint[material->drawables.size()];
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, visible_instances_ssbo);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, material->drawables.size() * sizeof(GLuint), visible_instances);

    // TODO: Pass visible instances directly to the shader by a shared SSBO. Might not actually be beneficial?
    for (uint32_t i = 0; i < material->drawables.size(); ++i)
    {
        if (visible_instances[i] == 1)
        {
            material->model_matrices.emplace_back(material->drawables[i]->entity->transform->get_model_matrix());
        }
    }

    // Free visible_instances memory
    delete[] visible_instances;

    material->shader->use();

    // Pass model matrices of visible instances to the GPU
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, gpu_instancing_ssbo);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, material->model_matrices.size() * sizeof(glm::mat4), material->model_matrices.data());
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}