#include <rendering/pipeline/default/stages/postprocessingstage.hpp>

namespace legion::rendering
{

    std::multimap<priority_type, std::unique_ptr<PostProcessingEffectBase>, std::greater<>> PostProcessingStage::m_effects;


    void PostProcessingStage::setup(app::window& context)
    {
        OPTICK_EVENT();
        using namespace legion::core::fs::literals;

        app::context_guard guard(context);

        m_screenQuad = screen_quad::generate();

        m_drawFBO = framebuffer(GL_FRAMEBUFFER);

        m_swapTexture = TextureCache::create_texture("color_swap_image", math::ivec2(1, 1), {
        texture_type::two_dimensional, channel_format::float_hdr, texture_format::rgba_hdr,
        texture_components::rgb, true, true, texture_mipmap::linear, texture_mipmap::linear,
        texture_wrap::repeat, texture_wrap::repeat, texture_wrap::repeat });

        m_screenShader = ShaderCache::create_shader("screen shader", "engine://shaders/screenshader.shs"_view);
    }

    void PostProcessingStage::render(app::window& context, camera& cam, const camera::camera_input& camInput, time::span deltaTime)
    {
        OPTICK_EVENT();
        static id_type mainId = nameHash("main");

        auto fbo = getFramebuffer(mainId);
        if (!fbo)
        {
            log::error("Main frame buffer is missing.");
            abort();
            return;
        }

        app::context_guard guard(context);
        if (!guard.contextIsValid())
        {
            abort();
            return;
        }

        auto [valid, message] = fbo->verify();
        if (!valid)
        {
            log::error("Main frame buffer isn't complete: {}", message);
            abort();
            return;
        }

        auto colorAttachment = fbo->getAttachment(GL_COLOR_ATTACHMENT0);
        if (std::holds_alternative<std::monostate>(colorAttachment))
        {
            log::error("Color attachment was not found.");
            return;
        }
        if (!std::holds_alternative<texture_handle>(colorAttachment))
        {
            log::error("Color attachment needs to be a texture to be able to use it for post processing.");
            return;
        }

        texture_handle textures[] = { std::get<texture_handle>(colorAttachment), m_swapTexture };

        math::ivec2 attachmentSize = textures[0].get_texture().size();
        auto tex = m_swapTexture.get_texture();
        if (attachmentSize != tex.size())
            tex.resize(attachmentSize);

        int index = 0;

        bool stencil = false;
        auto depthAttachment = fbo->getAttachment(GL_DEPTH);
        if (std::holds_alternative<std::monostate>(depthAttachment))
        {
            stencil = true;
            depthAttachment = fbo->getAttachment(GL_DEPTH_STENCIL);
        }

        texture_handle depthTexture = invalid_texture_handle;
        if (std::holds_alternative<texture_handle>(depthAttachment))
            depthTexture = std::get<texture_handle>(depthAttachment);
        glDisable(GL_DEPTH_TEST);


        for (auto& [_, effect] : m_effects)
        {
            OPTICK_EVENT("Rendering effect");
            OPTICK_TAG("Effect", effect->getName().c_str());

            if (!effect->isInitialized()) effect->init(context);
            for (auto& pass : effect->renderPasses)
            {
                OPTICK_EVENT("Effect pass");
                fbo->attach(textures[!index], GL_COLOR_ATTACHMENT0);
                
                pass.invoke(*fbo, textures[index], depthTexture, deltaTime);
                                
                index = !index;
            }
        }

        if (index)
        {
            fbo->attach(textures[0], GL_COLOR_ATTACHMENT0);
            fbo->bind();
            m_screenShader.bind();
            m_screenShader.get_uniform_with_location<texture_handle>(SV_SCENECOLOR).set_value(textures[1]);
            m_screenQuad.render();
            fbo->release();
        }

        rendering::shader::release();

        glEnable(GL_DEPTH_TEST);
    }

    priority_type PostProcessingStage::priority()
    {
        return post_fx_priority;
    }

}
