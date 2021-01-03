#pragma once
#include <core/platform/platform.hpp>
#include <core/engine/module.hpp>
#include <core/types/primitives.hpp>
#include <core/types/meta.hpp>
#include <core/ecs/ecsregistry.hpp>
#include <core/scheduling/scheduler.hpp>
#include <core/events/eventbus.hpp>
#include <core/defaults/coremodule.hpp>
#include <core/logging/logging.hpp>
#include <core/ecs/component_handle.hpp>

#include <map>
#include <vector>
#include <memory>

/**
 * @file engine.hpp
 */

namespace legion::core
{
    /**@class Engine
     * @brief Main top level engine abstraction.
     *        This class allows you to setup the engine with all the necessary modules and settings.
     *        This class also contains all the sub managers of the core systems.
     * @ref legion::core::Module
     * @ref legion::core::ecs::EcsRegistry
     * @ref legion::core::scenes::SceneManager
     * @ref legion::core::scheduling::Scheduler
     */
    class Engine
    {
    private:
        std::vector<char*> m_cliargs;
        std::map<priority_type, std::vector<std::unique_ptr<Module>>, std::greater<>> m_modules;

        events::EventBus m_eventbus;
        ecs::EcsRegistry m_ecs;
        scheduling::Scheduler m_scheduler;

    public:

        inline static events::EventBus* eventbus;

        Engine(int argc, char** argv) : m_modules(), m_eventbus(), m_ecs(&m_eventbus), m_cliargs(argv, argv + argc),
#if defined(LEGION_LOW_POWER)
            m_scheduler(&m_eventbus, true, LEGION_MIN_THREADS)
#else
            m_scheduler(&m_eventbus, false, LEGION_MIN_THREADS)
#endif
        {
            log::setup();
            eventbus = &m_eventbus;
            Module::m_eventBus = &m_eventbus;
            Module::m_ecs = &m_ecs;
            Module::m_scheduler = &m_scheduler;
            SystemBase::m_eventBus = &m_eventbus;
            SystemBase::m_ecs = &m_ecs;
            SystemBase::m_scheduler = &m_scheduler;
            ecs::component_handle_base::m_registry = &m_ecs;
            ecs::component_handle_base::m_eventBus = &m_eventbus;

            reportModule<CoreModule>();
        }

        ~Engine()
        {
            m_modules.clear();
        }

        /**@brief reports an engine module
         * @tparam ModuleType the module you want to report
         * @param s a signal that you want to pass arguments to the constructor of the Module
         * @param args the arguments you want to pass
         * @ref legion::core::Module
         */
        template <class ModuleType, class... Args, inherits_from<ModuleType, Module> = 0>
        void reportModule(Args&&...args)
        {
            std::unique_ptr<Module> module = std::make_unique<ModuleType>(std::forward<Args>(args)...);
            module->m_ecs = &m_ecs;
            module->m_scheduler = &m_scheduler;
            module->m_eventBus = &m_eventbus;

            const priority_type priority = module->priority();
            m_modules[priority].emplace_back(std::move(module));
        }

        void reportModule(std::unique_ptr<Module>&& module)
        {
            module->m_ecs = &m_ecs;
            module->m_scheduler = &m_scheduler;
            module->m_eventBus = &m_eventbus;

            const priority_type priority = module->priority();
            m_modules[priority].emplace_back(std::move(module));
        }

        /**@brief Calls init on all reported modules and thus engine internals.
         * @note Needs to be called manually if LEGION_ENTRY was not used.
         * @param argc argc of main
         * @param argv argv of main
         * @ref legion::core::Module
         */
        void init()
        {
            for (const auto& [priority, moduleList] : m_modules)
                for (auto& module : moduleList)
                    module->setup();

            for (const auto& [priority, moduleList] : m_modules)
                for (auto& module : moduleList)
                    module->init();
        }

        /**@brief Runs engine loop.
         */
        void run()
        {
            m_scheduler.run();
        }

        std::vector<char*>& getCliArgs() {
            return m_cliargs;
        }

    };
}
