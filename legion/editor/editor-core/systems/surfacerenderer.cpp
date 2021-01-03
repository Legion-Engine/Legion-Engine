#include <editor-core/systems/surfacerenderer.hpp>

namespace legion::editor
{

    std::unordered_map<id_type, size_type> SurfaceRenderer::m_surfaceCounts;
    std::unordered_map<id_type, std::unordered_set<id_type>> SurfaceRenderer::m_surfacesOfType;
    sparse_map<id_type, std::unique_ptr<EditorSurfaceBase>> SurfaceRenderer::m_surfaces;

    void SurfaceRenderer::setup()
    {
        gfx::ImGuiStage::addGuiRender<SurfaceRenderer, &SurfaceRenderer::render>(this);
    }

    std::pair<id_type, const std::string&> SurfaceRenderer::openSurface(std::unique_ptr<EditorSurfaceBase>&& surface)
    {
        surface->setup();
        size_type surfaceCount = ++m_surfaceCounts[surface->getTypeId()];
        if (surfaceCount > 1)
            surface->setName(surface->getName() + " " + std::to_string(surfaceCount));

        const std::string& name = surface->getName();
        id_type id = surface->getId();
        m_surfaces.emplace(id, surface.release());
        return std::make_pair(id, std::ref(name));
    }

    size_type SurfaceRenderer::closeAllSurfacesOfType(id_type typeId)
    {
        size_type count = m_surfaceCounts[typeId];
        m_surfaceCounts.erase(typeId);
        for (auto id : m_surfacesOfType[typeId])
        {
            m_surfaces.erase(id);
        }
        m_surfacesOfType.erase(typeId);

        return count;
    }

    size_type SurfaceRenderer::closeSurface(id_type id)
    {
        if (!m_surfaces.contains(id))
            return 0;

        id_type typeId = m_surfaces[id]->getTypeId();
        auto ret = m_surfaces.erase(id);
        if (ret)
        {
            m_surfacesOfType[typeId].erase(id);
            m_surfaceCounts[typeId]--;
        }

        return ret;
    }

    size_type SurfaceRenderer::closeSurface(const std::string& name)
    {
        id_type id = nameHash(name);
        if (!m_surfaces.contains(id))
            return 0;

        id_type typeId = m_surfaces[id]->getTypeId();
        auto ret = m_surfaces.erase(id);
        if (ret)
        {
            m_surfacesOfType[typeId].erase(id);
            m_surfaceCounts[typeId]--;
        }

        return ret;
    }

    void SurfaceRenderer::render(app::window& context, gfx::camera& cam, const gfx::camera::camera_input& camInput, time::span deltaTime)
    {
        static std::unordered_set<id_type> toErase;
        for (auto [id, surf] : m_surfaces)
        {
            if (!surf->drawSurface(context, cam, camInput, deltaTime))
                toErase.insert(id);
        }

        for (auto id : toErase)
            closeSurface(id);

        toErase.clear();
    }

}
