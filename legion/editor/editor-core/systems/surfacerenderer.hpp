#pragma once
#include <rendering/rendering.hpp>
#include <editor-core/tooling/base/editorsurface.hpp>

namespace legion::editor
{
    class SurfaceRenderer : public System<SurfaceRenderer>
    {
    private:
        static std::unordered_map<id_type, size_type> m_surfaceCounts;
        static std::unordered_map<id_type, std::unordered_set<id_type>> m_surfacesOfType;
        static sparse_map<id_type, std::unique_ptr<EditorSurfaceBase>> m_surfaces;

    public:
        virtual void setup() override;

        static std::pair<id_type, const std::string&> openSurface(std::unique_ptr<EditorSurfaceBase>&& surface);

        template<typename SurfaceType, typename... Args, inherits_from<SurfaceType, EditorSurface<SurfaceType>> = 0>
        static std::pair<id_type, const std::string&> openSurface(Args&&... args);

        static size_type closeAllSurfacesOfType(id_type typeId);
        static size_type closeSurface(id_type id);
        static size_type closeSurface(const std::string& name);

        template<typename SurfaceType, inherits_from<SurfaceType, EditorSurface<SurfaceType>> = 0>
        static size_type closeAllSurfacesOfType();

        void render(app::window& context, gfx::camera& cam, const gfx::camera::camera_input& camInput, time::span deltaTime);
    };
}

#include <editor-core/systems/surfacerenderer.inl>
