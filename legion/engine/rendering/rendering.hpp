#pragma once
#include <rendering/data/model.hpp>
#include <rendering/systems/renderer.hpp>
#include <rendering/module/renderingmodule.hpp>
#include <rendering/debugrendering.hpp>
#include <rendering/pipeline/default/stages/postprocessingstage.hpp>
#include <rendering/pipeline/default/postfx/tonemapping.hpp>
#include <rendering/data/buffer.hpp>
#include <rendering/data/framebuffer.hpp>
#include <rendering/data/renderbuffer.hpp>
#include <rendering/data/vertexarray.hpp>
#include <rendering/data/material.hpp>
#include <rendering/data/postprocessingeffect.hpp>
#include <rendering/data/particle_system_base.hpp>
#include <rendering/data/particle_system_cache.hpp>
#include <rendering/util/gui.hpp>
#include <rendering/pipeline/gui/stages/imguirenderstage.hpp>

namespace legion
{
    namespace gfx = ::legion::rendering;
}
