#include <editor-core/systems/menurenderer.hpp>

namespace legion::editor
{
    std::pair<menu_folder*, std::string> MenuRenderer::extractFolderAndName(const std::string& path)
    {
        if (path.empty())
            return { nullptr, std::string() };

        std::vector<std::string> dir = common::split_string_at<'\\', '/'>(path);
        if (dir.size() == 1)
            dir.insert(dir.begin(), "main");
        std::string& optionName = dir[dir.size() - 1];
        menu_folder* folder = &m_menus[dir[0]];
        for (int i = 1; i < dir.size() - 1; i++)
        {
            folder = &folder->subfolders[dir[i]];
        }
        return { folder, optionName };
    }

    void MenuRenderer::renderFolder(menu_folder* folder)
    {
        if (ImGui::BeginMenu(folder->name.c_str()))
        {
            for (auto& [_, subfolder] : folder->subfolders)
                renderFolder(&subfolder);

            for (auto& [optionName, option] : folder->menuOptions)
            {
                if (ImGui::MenuItem(optionName.c_str(), nullptr, nullptr, option->enabled))
                {
                    option->execute();
                }
                else if (!option->tooltip.empty() && ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::TextUnformatted(option->tooltip.c_str());
                    ImGui::EndTooltip();
                }
            }

            ImGui::EndMenu();
        }
    }

    void MenuRenderer::setup()
    {
        gfx::ImGuiStage::addGuiRender<MenuRenderer, &MenuRenderer::renderMainMenu>(this);
    }

    void MenuRenderer::addMenuOption(const std::string& name, delegate<void()>&& func, const std::string& tooltip)
    {
        auto [folder, optionName] = extractFolderAndName(name);
        folder->menuOptions.emplace(optionName, std::unique_ptr<menu_option_base>(new menu_option<delegate<void()>>(name, func, tooltip)));
    }

    void MenuRenderer::enableMenuOption(const std::string& name, bool enabled)
    {
        auto [folder, optionName] = extractFolderAndName(name);
        folder->menuOptions[optionName]->enabled = enabled;
    }

    void MenuRenderer::disableMenuOption(const std::string& name)
    {
        auto [folder, optionName] = extractFolderAndName(name);
        folder->menuOptions[optionName]->enabled = false;
    }

    void MenuRenderer::setToolTip(const std::string& name, const std::string& tooltip)
    {
        auto [folder, optionName] = extractFolderAndName(name);
        folder->menuOptions[optionName]->tooltip = tooltip;
    }

    void MenuRenderer::renderMenu(const std::string& name)
    {
        auto [folder, optionName] = extractFolderAndName(name + "/.");
        if (ImGui::BeginMenuBar())
        {
            for (auto& [_, subfolder] : folder->subfolders)
                renderFolder(&subfolder);

            for (auto& [optionName, option] : folder->menuOptions)
            {
                if (ImGui::MenuItem(optionName.c_str(), nullptr, nullptr, option->enabled))
                {
                    option->execute();
                }
                else if (!option->tooltip.empty() && ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::TextUnformatted(option->tooltip.c_str());
                    ImGui::EndTooltip();
                }
            }

            ImGui::EndMenuBar();
        }
    }

    void MenuRenderer::renderMainMenu(app::window& context, gfx::camera& cam, const gfx::camera::camera_input& camInput, time::span deltaTime)
    {
        auto& folder = m_menus["main"];
        if (ImGui::BeginMenuBar())
        {
            for (auto& [_, subfolder] : folder.subfolders)
                renderFolder(&subfolder);

            for (auto& [optionName, option] : folder.menuOptions)
            {
                if (ImGui::MenuItem(optionName.c_str(), nullptr, nullptr, option->enabled))
                {
                    option->execute();
                }
                else if (!option->tooltip.empty() && ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::TextUnformatted(option->tooltip.c_str());
                    ImGui::EndTooltip();
                }
            }

            ImGui::EndMenuBar();
        }
    }

}
