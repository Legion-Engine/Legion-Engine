#include <editor-core/tooling/base/editortool.hpp>

namespace legion::editor
{
    std::vector<menu_option_info> EditorToolBase::getMenuOptions()
    {
        std::vector<menu_option_info> menuItems;

        for (auto& [hash, ptr] : m_menuOptions)
            menuItems.push_back(std::make_tuple(ptr->name, std::ref(ptr->tooltip), ptr->enabled));

        return menuItems;
    }

    void EditorToolBase::addMenuOption(const std::string& name, delegate<void()>&& func, const std::string& tooltip)
    {
        id_type id = nameHash(name);

        m_menuOptions.emplace(id, std::unique_ptr<menu_option_base>(new menu_option<delegate<void()>>(name, std::forward<delegate<void()>&&>(func), tooltip)));
    }

    void EditorToolBase::enableMenuOption(const std::string& name, bool enabled)
    {
        id_type id = nameHash(name);
        if (m_menuOptions.count(id))
            m_menuOptions[id]->enabled = enabled;
    }

    void EditorToolBase::enableMenuOption(id_type id, bool enabled)
    {
        if (m_menuOptions.count(id))
            m_menuOptions[id]->enabled = enabled;
    }

    void EditorToolBase::setToolTip(const std::string& name, const std::string& tooltip)
    {
        id_type id = nameHash(name);
        if (m_menuOptions.count(id))
            m_menuOptions[id]->tooltip = tooltip;
    }

    void EditorToolBase::setToolTip(id_type id, const std::string& tooltip)
    {
        if (m_menuOptions.count(id))
            m_menuOptions[id]->tooltip = tooltip;
    }

    void EditorToolBase::openSurface(std::unique_ptr<EditorSurfaceBase>&& surface)
    {
        SurfaceRenderer::openSurface(std::forward<std::unique_ptr<EditorSurfaceBase>&&>(surface));
    }

}
