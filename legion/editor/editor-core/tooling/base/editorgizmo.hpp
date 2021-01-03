#pragma once
#include <rendering/rendering.hpp>

namespace legion::editor
{
    class EditorGizmo
    {
        virtual void onGizmo(app::window& context, gfx::camera& cam, const gfx::camera::camera_input& camInput, time::span deltaTime) LEGION_PURE;
    };
}
