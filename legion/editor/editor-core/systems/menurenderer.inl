#pragma once

namespace legion::editor
{
    template<typename FuncType>
    inline void MenuRenderer::addMenuOption(const std::string& name, FuncType func, const std::string& tooltip)
    {
        auto [folder, optionName] = extractFolderAndName(name);
        folder->menuOptions.emplace(optionName, std::unique_ptr<menu_option_base>(new menu_option<FuncType>(name, func, tooltip)));
    }
}
