#pragma once

namespace legion::editor
{
    template<typename SurfaceType, typename... Args, inherits_from<SurfaceType, EditorSurface<SurfaceType>>>
    inline std::pair<id_type, const std::string&> SurfaceRenderer::openSurface(Args&&... args)
    {
        SurfaceType* surface = new SurfaceType(std::forward<Args>(args)...);
        surface->setup();

        size_type surfaceCount = ++m_surfaceCounts[SurfaceType::surfaceType];
        if (surfaceCount > 1)
            surface->setName(surface->getName() + " " + std::to_string(surfaceCount));

        id_type id = surface->getId();
        m_surfaces.emplace(id, surface);
        return std::make_pair(id, std::ref(surface->getName()));
    }

    template<typename SurfaceType, inherits_from<SurfaceType, EditorSurface<SurfaceType>>>
    inline size_type SurfaceRenderer::closeAllSurfacesOfType()
    {
        closeAllSurfacesOfType(SurfaceType::surfaceType);
    }

}
