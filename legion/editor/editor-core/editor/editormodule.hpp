#pragma once
#include <core/core.hpp>

/**
 * @file editormodule.hpp
 */

namespace legion::editor
{
    namespace detail
    {
        class EditorEngineModule;
    }

    class EditorModuleBase
    {
        friend class Editor;
        friend class legion::editor::detail::EditorEngineModule;
    protected:
        virtual Module* getEngineModule() LEGION_PURE;

    public:
        virtual void setup() LEGION_PURE;
    };

    /**@class EditorModule
     * @brief interface for editor-modules, must be implemented
     * @ref legion::editor::Editor::reportModule<T,...>()
     */
    template<typename EngineModule>
    class EditorModule : public EditorModuleBase
    {
        friend class Editor;
        friend class legion::editor::detail::EditorEngineModule;
    private:
        Module* m_engineModule;

    protected:
        virtual Module* getEngineModule() override;

        template<typename... Args>
        void parameteriseEngineModule(Args&&... args);
    };

    class CoreEditorModule : public EditorModule<CoreModule>
    {
    public:
        virtual void setup();
    };

    template<typename T>
    Module* EditorModule<T>::getEngineModule()
    {
        if (!m_engineModule)
            m_engineModule = new EngineModule();
        return m_engineModule;
    }

    template<>
    inline Module* EditorModule<void>::getEngineModule()
    {
        return nullptr;
    }

    template<>
    inline Module* EditorModule<std::nullptr_t>::getEngineModule()
    {
        return nullptr;
    }

    template<typename T>
    template<typename... Args>
    void EditorModule<T>::parameteriseEngineModule(Args&&... args)
    {
        m_engineModule = new EngineModule(std::forward<Args>(args)...);
    }

    template<>
    template<typename... Args>
    void EditorModule<void>::parameteriseEngineModule(Args&&... args)
    {
    }

    template<>
    template<typename... Args>
    void EditorModule<std::nullptr_t>::parameteriseEngineModule(Args&&... args)
    {
    }
}
