#pragma once
#include <rendering/rendering.hpp>

namespace legion::editor
{
    class EditorSurfaceBase
    {
    protected:
        std::string m_name;
        id_type m_id;

    public:
        EditorSurfaceBase() = default;
        explicit EditorSurfaceBase(const std::string& name);

        virtual void setup() LEGION_PURE;

        id_type getId();
        virtual id_type getTypeId() LEGION_PURE;

        void setName(const std::string& name);
        const std::string& getName();

        bool drawSurface(app::window& context, gfx::camera& cam, const gfx::camera::camera_input& camInput, time::span deltaTime);
        virtual void onGUI(app::window& context, gfx::camera& cam, const gfx::camera::camera_input& camInput, time::span deltaTime) LEGION_PURE;
    };

    template<typename SelfType>
    class EditorSurface : public EditorSurfaceBase
    {
    protected:
        inline static size_type m_surfaceCount = 0;
    public:
        inline static const id_type surfaceType = typeHash<SelfType>();
        EditorSurface() : EditorSurfaceBase(std::string(typeName<SelfType>()) + (m_surfaceCount ? " " + std::to_string(m_surfaceCount + 1) : "")) { m_surfaceCount++; }
        explicit EditorSurface(const std::string& name) : EditorSurfaceBase(name) { m_surfaceCount++; }

        virtual id_type getTypeId()
        {
            return surfaceType;
        }
    };
}
