#pragma once
#include <core/core.hpp>
#include <rendering/rendering.hpp>
#include <rendering/util/gui.hpp>
#include <rendering/pipeline/gui/stages/imguirenderstage.hpp>

using namespace legion::rendering;

class GuiTestSystem : public System<GuiTestSystem>
{
    ecs::EntityQuery cameraQuery = createQuery<camera, position, rotation, scale>();


    material_handle vertexColorMaterial;
    model_handle cubeModel;
    ecs::entity_handle cubeEntity;


    char guiTextBuffer[512]{ 0 };

    math::mat4 view = math::mat4(1.0f);
    math::mat4 projection = math::mat4(1.0f);
    math::mat4 model = math::mat4(1.0f);


    void setup() override
    {

        static_cast<DefaultPipeline*>(Renderer::getMainPipeline())->attachStage<ImGuiStage>();

        app::window window = m_ecs->world.get_component_handle<app::window>().read();

        {
            application::context_guard guard(window);

            cubeModel = ModelCache::create_model("cube", "assets://models/cube.obj"_view);
            vertexColorMaterial = MaterialCache::create_material("color shader", "assets://shaders/texture.shs"_view);
        }


        cubeEntity = createEntity();

        cubeEntity.add_components<transform>(position(), rotation(), scale());
        cubeEntity.add_components<mesh_renderable>(mesh_filter(cubeModel.get_mesh()), mesh_renderer(vertexColorMaterial));


        //gui code goes here
        ImGuiStage::addGuiRender<GuiTestSystem,&GuiTestSystem::onGUI>(this);
        createProcess<&GuiTestSystem::update>("Update");
    }

    void onGUI(app::window& context, camera& cam, const camera::camera_input& camInput, time::span deltaTime)
    {
        static bool firstFrame = true;

        if (firstFrame)
        {
            firstFrame = false;
            ImVec4* colors = ImGui::GetStyle().Colors;
            colors[ImGuiCol_Text] = ImVec4(0.93f, 0.94f, 0.96f, 1.00f);
            colors[ImGuiCol_TextDisabled] = ImVec4(0.65f, 0.67f, 0.71f, 1.00f);
            colors[ImGuiCol_WindowBg] = ImVec4(0.18f, 0.20f, 0.25f, 1.00f);
            colors[ImGuiCol_ChildBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
            colors[ImGuiCol_PopupBg] = ImVec4(0.23f, 0.26f, 0.32f, 0.94f);
            colors[ImGuiCol_Border] = ImVec4(0.30f, 0.34f, 0.42f, 0.00f);
            colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
            colors[ImGuiCol_FrameBg] = ImVec4(0.26f, 0.30f, 0.37f, 0.54f);
            colors[ImGuiCol_FrameBgHovered] = ImVec4(0.23f, 0.26f, 0.32f, 0.40f);
            colors[ImGuiCol_FrameBgActive] = ImVec4(0.23f, 0.26f, 0.32f, 0.67f);
            colors[ImGuiCol_TitleBg] = ImVec4(0.18f, 0.20f, 0.25f, 1.00f);
            colors[ImGuiCol_TitleBgActive] = ImVec4(0.18f, 0.20f, 0.25f, 1.00f);
            colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.00f, 0.00f, 0.00f, 0.51f);
            colors[ImGuiCol_MenuBarBg] = ImVec4(0.23f, 0.26f, 0.32f, 1.00f);
            colors[ImGuiCol_ScrollbarBg] = ImVec4(0.23f, 0.26f, 0.32f, 1.00f);
            colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.30f, 0.34f, 0.42f, 1.00f);
            colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.26f, 0.30f, 0.37f, 1.00f);
            colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.26f, 0.30f, 0.37f, 1.00f);
            colors[ImGuiCol_CheckMark] = ImVec4(0.37f, 0.51f, 0.67f, 1.00f);
            colors[ImGuiCol_SliderGrab] = ImVec4(0.37f, 0.51f, 0.67f, 1.00f);
            colors[ImGuiCol_SliderGrabActive] = ImVec4(0.37f, 0.42f, 0.52f, 1.00f);
            colors[ImGuiCol_Button] = ImVec4(0.37f, 0.51f, 0.67f, 0.40f);
            colors[ImGuiCol_ButtonHovered] = ImVec4(0.37f, 0.42f, 0.52f, 1.00f);
            colors[ImGuiCol_ButtonActive] = ImVec4(0.37f, 0.42f, 0.52f, 1.00f);
            colors[ImGuiCol_Header] = ImVec4(0.37f, 0.51f, 0.67f, 0.40f);
            colors[ImGuiCol_HeaderHovered] = ImVec4(0.37f, 0.42f, 0.52f, 1.00f);
            colors[ImGuiCol_HeaderActive] = ImVec4(0.37f, 0.42f, 0.52f, 1.00f);
            colors[ImGuiCol_Separator] = ImVec4(0.23f, 0.26f, 0.32f, 1.00f);
            colors[ImGuiCol_SeparatorHovered] = ImVec4(0.23f, 0.26f, 0.32f, 1.00f);
            colors[ImGuiCol_SeparatorActive] = ImVec4(0.23f, 0.26f, 0.32f, 1.00f);
            colors[ImGuiCol_ResizeGrip] = ImVec4(0.30f, 0.34f, 0.42f, 0.80f);
            colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.30f, 0.34f, 0.42f, 1.00f);
            colors[ImGuiCol_ResizeGripActive] = ImVec4(0.30f, 0.34f, 0.42f, 1.00f);
            colors[ImGuiCol_Tab] = ImVec4(0.30f, 0.34f, 0.42f, 0.80f);
            colors[ImGuiCol_TabHovered] = ImVec4(0.37f, 0.42f, 0.52f, 1.00f);
            colors[ImGuiCol_TabActive] = ImVec4(0.37f, 0.51f, 0.67f, 0.40f);
            colors[ImGuiCol_TabUnfocused] = ImVec4(0.23f, 0.26f, 0.32f, 0.97f);
            colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.30f, 0.34f, 0.42f, 0.97f);
            colors[ImGuiCol_PlotLines] = ImVec4(0.53f, 0.75f, 0.82f, 1.00f);
            colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.71f, 0.56f, 0.68f, 1.00f);
            colors[ImGuiCol_PlotHistogram] = ImVec4(0.53f, 0.75f, 0.82f, 1.00f);
            colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.71f, 0.56f, 0.68f, 1.00f);
            colors[ImGuiCol_TextSelectedBg] = ImVec4(0.37f, 0.51f, 0.67f, 0.47f);
            colors[ImGuiCol_DragDropTarget] = ImVec4(0.71f, 0.56f, 0.68f, 0.90f);
            colors[ImGuiCol_NavHighlight] = ImVec4(0.64f, 0.75f, 0.55f, 1.00f);
            colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
            colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.80f, 0.80f, 0.80f, 0.20f);
            colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.80f, 0.80f, 0.80f, 0.35f);

            ImGuiStyle& style = ImGui::GetStyle();
            style.WindowRounding = 2.f;
            style.ChildRounding = 1.f;
            style.FrameRounding = 1.f;
            style.FrameRounding = 1.f;
            style.PopupRounding = 1.f;
            style.ScrollbarRounding = 2.f;
            style.GrabRounding = 1.f;
            style.TabRounding = 2.f;
        }

        ImGuiIO& io = ImGui::GetIO();

        setProjectionAndView(io.DisplaySize.x/io.DisplaySize.y, cam, camInput);


        using namespace imgui;
        base::ShowDemoWindow();
        gizmo::SetOrthographic(false);
        gizmo::BeginFrame();
        base::Begin("Hello World");
        gizmo::EditTransform(value_ptr(view), value_ptr(projection), value_ptr(model), true);
        base::End();

        base::Begin("Window");
        base::Text("Hello World!");


        base::End();


        //gizmo::ViewManipulate(value_ptr(view), 1.0f, ImVec2(io.DisplaySize.x - 128, 0), ImVec2(128, 128), 0x10101010);
    }



    void update(time::span dt)
    {

        if (cubeEntity.valid())
        {
            auto [mposh, mroth, mscaleh] = cubeEntity.get_component_handles<transform>();
            math::vec3 mpos, mscale;
            math::quat mrot;
            decompose(model, mscale, mrot, mpos);
            mposh.write(mpos);
            mroth.write(mrot);
            mscaleh.write(mscale);
        }

    }

    void setProjectionAndView(float aspect, const camera& cam, const camera::camera_input& camInput)
    {
        view = camInput.view;
        projection = math::perspective(math::deg2rad(cam.fov*aspect),aspect,cam.nearz,cam.farz);
    }
};

