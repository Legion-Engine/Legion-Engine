#pragma once
#include <core/core.hpp>

namespace legion::editor
{
    struct menu_option_base
    {
        const std::string name;
        std::string tooltip;
        bool enabled = true;

        menu_option_base(const std::string& menuName, const std::string& tooltip) : name(menuName), tooltip(tooltip) {}

        virtual void execute() LEGION_PURE;
    };

    template<typename executable>
    struct menu_option : public menu_option_base
    {
    private:
        executable m_exec;

    public:
        menu_option(const std::string& name, executable&& exec, const std::string& tooltip = "") : menu_option_base(name, tooltip), m_exec(exec) {}
        menu_option(const std::string& name, const executable& exec, const std::string& tooltip = "") : menu_option_base(name, tooltip), m_exec(exec) {}

        virtual void execute() override
        {
            m_exec();
        }
    };
}
