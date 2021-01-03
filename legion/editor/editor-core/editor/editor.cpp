#include <editor-core/editor/editor.hpp>
#include <rendering/rendering.hpp>

/**
 * @file editor.cpp
 */

namespace legion::editor
{
    Engine* Editor::m_engine;

    void Editor::setEngine(Engine* engine)
    {
        m_engine = engine;
    }

    Engine* Editor::getEngine()
    {
        return m_engine;
    }

    namespace detail
    {
        EditorEngineModule::EditorEngineModule(Editor* editor)
        {
            gfx::Renderer::getMainPipeline()->injectStage<gfx::ImGuiStage>();
            for (auto& module : editor->m_modules)
            {
                editor->m_engine->reportModule(std::unique_ptr<Module>(module->getEngineModule()));
            }
        }

        void EditorEngineModule::setup()
        {
        }
    }
}
