#pragma once
#include <rendering/rendering.hpp>
#include <editor-core/data/menu_folder.hpp>
#include <editor-core/data/menu_option.hpp>

namespace legion::editor
{
    class MenuRenderer : public System<MenuRenderer>
    {
    private:
        static std::unordered_map<std::string, menu_folder> m_menus;

        static std::pair<menu_folder*, std::string> extractFolderAndName(const std::string& path);

        static void renderFolder(menu_folder* folder);

    public:
        virtual void setup() override;

        template<typename FuncType>
        static void addMenuOption(const std::string& name, FuncType func, const std::string& tooltip = "");
        static void addMenuOption(const std::string& name, delegate<void()>&& func, const std::string& tooltip = "");
        static void enableMenuOption(const std::string& name, bool enabled = true);
        static void disableMenuOption(const std::string& name);
        static void setToolTip(const std::string& name, const std::string& tooltip);

        static void renderMenu(const std::string& name);

        void renderMainMenu(app::window& context, gfx::camera& cam, const gfx::camera::camera_input& camInput, time::span deltaTime);
    };
}

#include <editor-core/systems/menurenderer.inl>
