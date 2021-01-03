#include <rendering/pipeline/default/stages/debugrenderstage.hpp>

namespace legion::rendering
{
    async::spinlock DebugRenderStage::debugLinesLock;
    thread_local std::unordered_set<debug::debug_line>* DebugRenderStage::localLines;
    std::unordered_map<std::thread::id, std::unordered_set<debug::debug_line>*> DebugRenderStage::debugLines;

    void DebugRenderStage::startDebugDomain()
    {
        if (!localLines)
            localLines = new std::unordered_set<debug::debug_line>();
    }

    void DebugRenderStage::endDebugDomain()
    {
        size_type size = localLines->size();

        if (size == 0)
            return;

        std::thread::id id = std::this_thread::get_id();

        {
            std::lock_guard guard(debugLinesLock);

            if (debugLines[id])
            {
                for (auto& line : *(debugLines[id]))
                {
                    if (line.time > 0 && !localLines->count(line))
                        localLines->insert(line);
                }

                delete debugLines[id];
            }

            debugLines[id] = localLines;
            localLines = nullptr;
        }

        localLines = new std::unordered_set<debug::debug_line>();
        localLines->reserve(size);
    }

    void DebugRenderStage::drawDebugLine(events::event_base* event)
    {
        debug::debug_line* line = reinterpret_cast<debug::debug_line*>(event);
        if (localLines->count(*line))
            localLines->erase(*line);
        localLines->insert(*line);
    }

    void DebugRenderStage::setup(app::window& context)
    {
        scheduling::ProcessChain::subscribeToChainStart<&DebugRenderStage::startDebugDomain>();
        scheduling::ProcessChain::subscribeToChainEnd<&DebugRenderStage::endDebugDomain>();
        m_eventBus->bindToEventUnsafe(nameHash("debug_line"), delegate<void(events::event_base*)>::template create<DebugRenderStage, &DebugRenderStage::drawDebugLine>(this));
    }

    void DebugRenderStage::render(app::window& context, camera& cam, const camera::camera_input& camInput, time::span deltaTime)
    {
        using namespace legion::core::fs::literals;

        std::vector<debug::debug_line> lines;

        {
            std::lock_guard guard(debugLinesLock);
            if (debugLines.size() == 0)
                return;

            std::vector<debug::debug_line> toRemove;
            for (auto& [threadId, domain] : debugLines)
            {
                lines.insert(lines.end(), domain->begin(), domain->end());

                for (auto& line : (*domain))
                {
                    if (line.time == 0)
                        continue;

                    line.timeBuffer += deltaTime;

                    if (line.timeBuffer >= line.time)
                        toRemove.push_back(line);
                }

                for (auto line : toRemove)
                    domain->erase(line);
            }
        }

        static id_type mainId = nameHash("main");
        auto fbo = getFramebuffer(mainId);
        if (!fbo)
        {
            log::error("Main frame buffer is missing.");
            abort();
            return;
        }

        app::context_guard guard(context);
        if (!guard.contextIsValid())
        {
            abort();
            return;
        }

        static material_handle debugMaterial = MaterialCache::create_material("debug", "assets://shaders/debug.shs"_view);
        static app::gl_id vertexBuffer = -1;
        static size_type vertexBufferSize = 0;
        static app::gl_id colorBuffer = -1;
        static size_type colorBufferSize = 0;
        static app::gl_id vao = -1;

        if (debugMaterial == invalid_material_handle)
            return;

        if (vertexBuffer == -1)
            glGenBuffers(1, &vertexBuffer);

        if (colorBuffer == -1)
            glGenBuffers(1, &colorBuffer);

        if (vao == -1)
            glGenVertexArrays(1, &vao);

        std::unordered_map<bool, std::unordered_map<float, std::pair<std::vector<math::color>, std::vector<math::vec3>>>> lineBatches;

        for (auto& line : lines)
        {
            auto& [colors, vertices] = lineBatches[line.ignoreDepth][line.width];

            colors.push_back(line.color);
            colors.push_back(line.color);
            vertices.push_back(line.start);
            vertices.push_back(line.end);
        }

        auto [valid, message] = fbo->verify();
        if (!valid)
        {
            log::error("Main frame buffer isn't complete: {}", message);
            abort();
            return;
        }

        fbo->bind();

        debugMaterial.bind();

        glEnable(GL_LINE_SMOOTH);
        glBindVertexArray(vao);

        for (auto& [ignoreDepth, widthNdata] : lineBatches)
            for (auto& [width, lineData] : widthNdata)
            {
                auto& [colors, vertices] = lineData;

                ///------------ vertices ------------///
                glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);

                size_type vertexCount = vertices.size();
                if (vertexCount > vertexBufferSize)
                {
                    glBufferData(GL_ARRAY_BUFFER, vertexCount * sizeof(math::vec3), 0, GL_DYNAMIC_DRAW);
                    vertexBufferSize = vertexCount;
                }

                glBufferSubData(GL_ARRAY_BUFFER, 0, vertexCount * sizeof(math::vec3), vertices.data());
                glEnableVertexAttribArray(SV_POSITION);
                glVertexAttribPointer(SV_POSITION, 3, GL_FLOAT, GL_FALSE, 0, 0);

                ///------------ colors ------------///
                glBindBuffer(GL_ARRAY_BUFFER, colorBuffer);

                size_type colorCount = colors.size();
                if (colorCount > colorBufferSize)
                {
                    glBufferData(GL_ARRAY_BUFFER, colorCount * sizeof(math::color), 0, GL_DYNAMIC_DRAW);
                    colorBufferSize = colorCount;
                }

                glBufferSubData(GL_ARRAY_BUFFER, 0, colorCount * sizeof(math::color), colors.data());

                auto colorAttrib = debugMaterial.get_attribute("color");

                if (colorAttrib == invalid_attribute)
                {
                    glBindBuffer(GL_ARRAY_BUFFER, 0);
                    break;
                }

                colorAttrib.set_attribute_pointer(4, GL_FLOAT, GL_FALSE, 0, 0);

                ///------------ camera ------------///
                glUniformMatrix4fv(SV_VIEW, 1, false, math::value_ptr(camInput.view));
                glUniformMatrix4fv(SV_PROJECT, 1, false, math::value_ptr(camInput.proj));

                glLineWidth(width + 1);

                if (ignoreDepth)
                    glDisable(GL_DEPTH_TEST);

                glDrawArraysInstanced(GL_LINES, 0, vertices.size(), colors.size());

                if (ignoreDepth)
                    glEnable(GL_DEPTH_TEST);
                colorAttrib.disable_attribute_pointer();
                glBindBuffer(GL_ARRAY_BUFFER, 0);
            }

        glBindVertexArray(0);

        glDisable(GL_LINE_SMOOTH);

        debugMaterial.release();

        fbo->release();
    }

    priority_type DebugRenderStage::priority()
    {
        return post_fx_priority + 1;
    }

}
