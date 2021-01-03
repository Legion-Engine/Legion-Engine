#pragma once
#include <editor-core/tooling/base/editortool.hpp>

namespace legion::editor
{
    class ViewportTool : public EditorTool<ViewportTool>
    {
        virtual void setup() override;

        void openNewViewport();
    };
}
