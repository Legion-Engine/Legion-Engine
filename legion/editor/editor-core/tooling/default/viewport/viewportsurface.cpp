#include <editor-core/tooling/default/viewport/viewportsurface.hpp>

namespace legion::editor
{
    void ViewportSurface::setup()
    {
        std::string name = "viewport";
        if (m_surfaceCount)
            name += " " + std::to_string(m_surfaceCount + 1);

        setName(name);
    }

    void ViewportSurface::onGUI(app::window& context, gfx::camera& cam, const gfx::camera::camera_input& camInput, time::span deltaTime)
    {
        // somehow get the result from the camera and render it as a texture to the imgui surface.
        // render all gizmos
    }

}
