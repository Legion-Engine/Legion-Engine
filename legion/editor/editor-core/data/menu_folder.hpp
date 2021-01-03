#pragma once
#include <editor-core/data/menu_option.hpp>

namespace legion::editor
{
    struct menu_folder
    {
        const std::string name;
        std::unordered_map<std::string, menu_folder> subfolders;
        std::unordered_map<std::string, std::unique_ptr<menu_option_base>> menuOptions;
    };
}
