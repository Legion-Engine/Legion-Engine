#pragma once
#include <editor-core/tooling/base/editorsurface.hpp>
#include <editor-core/tooling/base/editorgizmo.hpp>
#include <editor-core/data/menu_option.hpp>
#include <editor-core/systems/surfacerenderer.hpp>

namespace legion::editor
{
    using menu_option_info = std::tuple<std::string, std::string&, bool>;

    class EditorToolBase
    {
        /*template<typename SelfType>
        friend class EditorTool<SelfType>;*/
    public:
        virtual void setup() LEGION_PURE;

        std::vector<menu_option_info> getMenuOptions();
    protected:
        template<typename FuncType>
        void addMenuOption(const std::string& name, FuncType func, const std::string& tooltip = "")
        {
            id_type id = nameHash(name);

            m_menuOptions.emplace(id, std::unique_ptr<menu_option_base>(new menu_option<FuncType>(name, std::forward<FuncType>(func), tooltip)));
        }
        void addMenuOption(const std::string& name, delegate<void()>&& func, const std::string& tooltip = "");

        void enableMenuOption(const std::string& name, bool enabled = true);
        void enableMenuOption(id_type id, bool enabled = true);
        void disableMenuOption(const std::string& name) { enableMenuOption(name, false); }
        void disableMenuOption(id_type id) { enableMenuOption(id, false); }

        void setToolTip(const std::string& name, const std::string& tooltip);
        void setToolTip(id_type id, const std::string& tooltip);

        void openSurface(std::unique_ptr<EditorSurfaceBase>&& surface);

        template<typename SurfaceType, typename... Args, inherits_from<SurfaceType, EditorSurface<SurfaceType>> = 0>
        void openSurface(Args&&... args)
        {
            SurfaceRenderer::openSurface<SurfaceType>(std::forward<Args>(args)...);
        }

    //private:
        std::unordered_map<id_type, std::unique_ptr<menu_option_base>> m_menuOptions;
    };

    template<typename SelfType>
    class EditorTool : public EditorToolBase
    {
    protected:
        template<void(SelfType::* func_type)()>
        void addMenuOption(const std::string& name, const std::string& tooltip = "")
        {
            id_type id = nameHash(name);
            m_menuOptions.emplace(id, std::unique_ptr<menu_option_base>(new menu_option<delegate<void()>>(name, delegate<void()>::create<SelfType, func_type>((SelfType*)this), tooltip)));            
        }
    };
}
