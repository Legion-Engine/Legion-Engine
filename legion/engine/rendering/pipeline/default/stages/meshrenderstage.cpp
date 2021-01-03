#include <rendering/pipeline/default/stages/meshrenderstage.hpp>
#include <rendering/components/light.hpp>
#include <rendering/data/buffer.hpp>

namespace legion::rendering
{
    void MeshRenderStage::setup(app::window& context)
    {
    }

    void MeshRenderStage::render(app::window& context, camera& cam, const camera::camera_input& camInput, time::span deltaTime)
    {
        OPTICK_EVENT();
        (void)deltaTime;
        (void)cam;
        static id_type mainId = nameHash("main");
        static id_type batchesId = nameHash("mesh batches");
        static id_type lightsId = nameHash("light buffer");
        static id_type lightCountId = nameHash("light count");
        static id_type matricesId = nameHash("model matrix buffer");

        auto* batches = get_meta<sparse_map<material_handle, sparse_map<model_handle, std::unordered_set<ecs::entity_handle>>>>(batchesId);
        if (!batches)
            return;

        buffer* lightsBuffer = get_meta<buffer>(lightsId);
        if (!lightsBuffer)
            return;

        size_type* lightCount = get_meta<size_type>(lightCountId);
        if (!lightCount)
            return;

        buffer* modelMatrixBuffer = get_meta<buffer>(matricesId);
        if (!modelMatrixBuffer)
            return;

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

        auto [valid, message] = fbo->verify();
        if (!valid)
        {
            log::error("Main frame buffer isn't complete: {}", message);
            abort();
            return;
        }

        fbo->bind();

        for (auto [material, instancesPerMaterial] : *batches)
        {
            OPTICK_EVENT("Rendering material");
            auto materialName = material.get_name();
            OPTICK_TAG("Material", materialName.c_str());

            camInput.bind(material);
            if (material.has_param<uint>(SV_LIGHTCOUNT))
                material.set_param<uint>(SV_LIGHTCOUNT, *lightCount);

            material.bind();

            for (auto [modelHandle, instances] : instancesPerMaterial)
            {
                OPTICK_EVENT("Rendering instances");
                auto modelName = ModelCache::get_model_name(modelHandle.id);
                OPTICK_TAG("Model", modelName.c_str());

                if (!modelHandle.is_buffered())
                    modelHandle.buffer_data(*modelMatrixBuffer);

                model mesh = modelHandle.get_model();
                if (mesh.submeshes.empty())
                {
                    log::warn("Empty mesh found.");
                    continue;
                }

                {
                    OPTICK_EVENT("Calculating matrices");
                    m_matrices.resize(instances.size());
                    int i = 0;
                    for (auto& ent : instances)
                    {
                        m_matrices[i] = transform(ent.get_component_handles<transform>()).get_local_to_world_matrix();
                        i++;
                    }
                }
                modelMatrixBuffer->bufferData(m_matrices);

                mesh.vertexArray.bind();
                mesh.indexBuffer.bind();
                lightsBuffer->bind();
                for (auto submesh : mesh.submeshes)
                    glDrawElementsInstanced(GL_TRIANGLES, (GLuint)submesh.indexCount, GL_UNSIGNED_INT, (GLvoid*)(submesh.indexOffset * sizeof(uint)), (GLsizei)instances.size());

                lightsBuffer->release();
                mesh.indexBuffer.release();
                mesh.vertexArray.release();
            }

            material.release();
        }

        fbo->release();
    }

    priority_type MeshRenderStage::priority()
    {
        return opaque_priority;
    }
}
