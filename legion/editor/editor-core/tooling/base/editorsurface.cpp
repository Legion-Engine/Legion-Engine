#include <editor-core/tooling/base/editorsurface.hpp>

namespace legion::editor
{
    EditorSurfaceBase::EditorSurfaceBase(const std::string& name) : m_name(name), m_id(nameHash(m_name))
    {
    }

    id_type EditorSurfaceBase::getId()
    {
        return m_id;
    }

    void EditorSurfaceBase::setName(const std::string& name)
    {
        m_name = name;
        m_id = nameHash(name);
    }

    const std::string& EditorSurfaceBase::getName()
    {
        return m_name;
    }

    bool EditorSurfaceBase::drawSurface(app::window& context, gfx::camera& cam, const gfx::camera::camera_input& camInput, time::span deltaTime)
    {
        bool shouldNotClose = true;
        if (ImGui::Begin(m_name.c_str(), &shouldNotClose))
            onGUI(context, cam, camInput, deltaTime);
        ImGui::End();

        return shouldNotClose;
    }
}
