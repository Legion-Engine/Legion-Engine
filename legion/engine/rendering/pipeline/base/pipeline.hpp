#pragma once
#include <rendering/data/framebuffer.hpp>
#include <rendering/components/camera.hpp>
#include <rendering/pipeline/base/pipelinebase.hpp>
#include <rendering/pipeline/base/renderstage.hpp>
#include <application/application.hpp>

#include <memory>
#include <any>

namespace legion::rendering
{
    template<typename Self>
    class RenderPipeline : public RenderPipelineBase
    {
        friend class Renderer;
    protected:
        static std::multimap<priority_type, std::unique_ptr<RenderStageBase>, std::greater<>> m_stages;

        virtual void injectStageImpl(std::unique_ptr<RenderStageBase>&& stage) override { attachStage(std::forward<std::unique_ptr<RenderStageBase>&&>(stage)); }

    public:
        template<typename StageType, inherits_from<StageType, RenderStage<StageType>> = 0>
        static void attachStage();

        static void attachStage(std::unique_ptr<RenderStageBase>&& stage);

        virtual void setup(app::window& context) LEGION_PURE;

        void init(app::window& context) override;
        void render(app::window& context, camera& cam, const camera::camera_input& camInput, time::span deltaTime) override;
    };
}

#include <rendering/pipeline/base/pipeline.inl>
