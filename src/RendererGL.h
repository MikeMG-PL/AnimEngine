#pragma once

#include <glad/glad.h>

#include "AK/Badge.h"
#include "Renderer.h"

class RendererGL final : public Renderer
{
public:
    static std::shared_ptr<RendererGL> create();
    explicit RendererGL(AK::Badge<RendererGL>);

    ~RendererGL() override = default;

    virtual void begin_frame() const override;
    virtual void render_shadow_maps() const override;

    virtual void set_rasterizer_draw_type(RasterizerDrawType const rasterizer_draw_type) override;
    virtual void restore_default_rasterizer_draw_type() override;

protected:
    virtual void update_shader(std::shared_ptr<Shader> const& shader, glm::mat4 const& projection_view,
                               glm::mat4 const& projection_view_no_translation) const override;
    virtual void update_material(std::shared_ptr<Material> const& material) const override;
    virtual void update_object(std::shared_ptr<Drawable> const& drawable, std::shared_ptr<Material> const& material,
                               glm::mat4 const& projection_view, glm::mat4 const* bones = nullptr) const override;

    virtual void unbind_material(std::shared_ptr<Material> const& material) const override;

private:
    virtual void initialize_global_renderer_settings() override;
    virtual void initialize_buffers(size_t const max_size) override;
    virtual void perform_frustum_culling(std::shared_ptr<Material> const& material) const override;

    std::shared_ptr<Shader> m_frustum_culling_shader = {};

    GLuint m_gpu_instancing_ssbo = {};
    GLuint m_bounding_boxes_ssbo = {};
    GLuint m_visible_instances_ssbo = {};
};
