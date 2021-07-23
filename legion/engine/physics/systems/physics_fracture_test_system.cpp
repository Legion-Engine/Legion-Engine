#include <physics/systems/physics_fracture_test_system.hpp>
#include <physics/mesh_splitter_utils/mesh_splitter.hpp>
#include <physics/mesh_splitter_utils/splittable_polygon.hpp>
#include <physics/components/physics_component.hpp>
#include <physics/systems/physicssystem.hpp>
#include <physics/components/fracturer.hpp>
#include <rendering/debugrendering.hpp>
#include <rendering/components/camera.hpp>
#include <rendering/rendering.hpp>
#include <physics/components/fracturecountdown.hpp>
namespace legion::physics
{

    void PhysicsFractureTestSystem::setup()
    {
        using namespace legion::core::fs::literals;

        physics::PrimitiveMesh::SetECSRegistry(m_ecs);

        auto win = world.read_component<app::window>();

        app::context_guard guard(win);

        #pragma region Material Setup
   
        auto litShader = rendering::ShaderCache::create_shader("lit", fs::view("engine://shaders/default_lit.shs"));
        vertexColor = rendering::MaterialCache::create_material("vertexColor", "assets://shaders/vertexcolor.shs"_view);
        textureH = rendering::MaterialCache::create_material("texture", "assets://shaders/texture.shs"_view);
        textureH.set_param("_texture", rendering::TextureCache::create_texture("assets://textures/split-test.png"_view));

        ///////////textureH
        //textureH = rendering::MaterialCache::create_material("texture", litShader);
        ////textureH.set_variant("double_sided");
        //textureH.set_param("alphaCutoff", 0.0f);
        //textureH.set_param("useAlbedoTex", true);
        //textureH.set_param("useRoughnessTex", true);
        //textureH.set_param("useNormal", true);

        //textureH.set_param("useEmissiveTex", false);
        //textureH.set_param("useAmbientOcclusion", false);
        //textureH.set_param("useHeight", false);
        //textureH.set_param("useMetallicTex", false);
        //textureH.set_param("useMetallicRoughness", false);

        //textureH.set_param("metallicValue", 0.0f);
        //textureH.set_param("emissiveColor", math::colors::black);

        //textureH.set_param("albedoTex", rendering::TextureCache::create_texture("assets://textures/tile/tileColor.png"_view));
        //textureH.set_param("normalTex", rendering::TextureCache::create_texture("assets://textures/tile/tileNormal.png"_view));
        //textureH.set_param("roughnessTex", rendering::TextureCache::create_texture("assets://textures/tile/tileRoughness.png"_view));
        //textureH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

        /////////////////
        woodTextureH = rendering::MaterialCache::create_material("texture2", "assets://shaders/texture.shs"_view);
        woodTextureH.set_param("_texture", rendering::TextureCache::create_texture("assets://textures/log.jpg"_view));

        rockTextureH = rendering::MaterialCache::create_material("rock", "assets://shaders/texture.shs"_view);
        rockTextureH.set_param("_texture", rendering::TextureCache::create_texture("assets://textures/rock.png"_view));

        concreteH = rendering::MaterialCache::create_material("concrete", "assets://shaders/texture.shs"_view);
        concreteH.set_param("_texture", rendering::TextureCache::create_texture("assets://textures/concrete.png"_view));

        brickH = rendering::MaterialCache::create_material("brick", "assets://shaders/texture.shs"_view);
        brickH.set_param("_texture", rendering::TextureCache::create_texture("assets://textures/plaster/plasterColor.png"_view));

        //auto litShader = rendering::ShaderCache::create_shader("lit", fs::view("engine://shaders/default_lit.shs"));
       /* tileH = rendering::MaterialCache::create_material("tile", "assets://shaders/texture.shs"_view);
        tileH.set_param("_texture", rendering::TextureCache::create_texture("assets://textures/tile/tileColor.png"_view)); */

        //log::debug("------------------------------ TILE -------------");
 
        tileH = rendering::MaterialCache::create_material("tile", litShader);
        tileH.set_param("alphaCutoff", 0.5f);
        tileH.set_param("useAlbedoTex", true);
        tileH.set_param("useRoughnessTex", true);
        tileH.set_param("useNormal", true);

        tileH.set_param("useEmissiveTex", false);
        tileH.set_param("useAmbientOcclusion", false);
        tileH.set_param("useHeight", false);
        tileH.set_param("useMetallicTex", false);
        tileH.set_param("useMetallicRoughness", false);

        tileH.set_param("metallicValue", 0.0f);
        tileH.set_param("emissiveColor", math::colors::black);

        tileH.set_param("albedoTex", rendering::TextureCache::create_texture("assets://textures/tile/tileColor.png"_view));
        tileH.set_param("normalTex", rendering::TextureCache::create_texture("assets://textures/tile/tileNormal.png"_view));
        tileH.set_param("roughnessTex", rendering::TextureCache::create_texture("assets://textures/tile/tileRoughness.png"_view));
        tileH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

        wireFrameH = rendering::MaterialCache::create_material("wireframe", "assets://shaders/wireframe.shs"_view);
        #pragma endregion

        #pragma region Model Setup
        directionalLightH = rendering::ModelCache::create_model("directional light", "assets://models/directional-light.obj"_view);
        cubeH = rendering::ModelCache::create_model("cube", "assets://models/cube.obj"_view);
        planeH = rendering::ModelCache::create_model("plane", "assets://models/plane.obj"_view);
        cylinderH = rendering::ModelCache::create_model("cylinder", "assets://models/cylinder.obj"_view);
        concaveTestObject = rendering::ModelCache::create_model("concaveTestObject", "assets://models/polygonTest.obj"_view);
        colaH = rendering::ModelCache::create_model("cola", "assets://models/cola.glb"_view);
        hammerH = rendering::ModelCache::create_model("hammer", "assets://models/hammer.obj"_view);
        suzzaneH = rendering::ModelCache::create_model("suzanne", "assets://models/suzanne.glb"_view);
        teapotH = rendering::ModelCache::create_model("sah", "assets://models/gnomecentered.obj"_view);

        #pragma endregion

        #pragma region Input binding
        app::InputSystem::createBinding<physics_split_test>(app::inputmap::method::ENTER);
        app::InputSystem::createBinding<extendedPhysicsContinue>(app::inputmap::method::M);
        app::InputSystem::createBinding<nextPhysicsTimeStepContinue>(app::inputmap::method::N);
        //app::InputSystem::createBinding<spawnEntity>(app::inputmap::method::MOUSE_LEFT);
        app::InputSystem::createBinding<explosion>(app::inputmap::method::B);
        app::InputSystem::createBinding<QHULL>(app::inputmap::method::Q);
        app::InputSystem::createBinding<AddRigidbody>(app::inputmap::method::R);

       /* app::InputSystem::createBinding<smallExplosion>(app::inputmap::method::NUM1);
        app::InputSystem::createBinding<mediumExplosion>(app::inputmap::method::NUM2);
        app::InputSystem::createBinding<largeExplosion>(app::inputmap::method::NUM3);*/
      
        /*app::InputSystem::createBinding< activate_CRtest2>(app::inputmap::method::KP_2);
        app::InputSystem::createBinding< activate_CRtest3>(app::inputmap::method::KP_3);*/
        #pragma endregion

        #pragma region Function binding
        /*bindToEvent< activate_CRtest2, &TestSystem::onActivateUnitTest2>();
        bindToEvent< activate_CRtest3, &TestSystem::onActivateUnitTest3>();*/
        bindToEvent<physics_split_test,&PhysicsFractureTestSystem::OnSplit > ();
        bindToEvent<extendedPhysicsContinue, &PhysicsFractureTestSystem::extendedContinuePhysics>();
        bindToEvent<nextPhysicsTimeStepContinue, &PhysicsFractureTestSystem::OneTimeContinuePhysics>();
        bindToEvent<spawnEntity, &PhysicsFractureTestSystem::spawnEntityOnCameraForward>();

        bindToEvent<smallExplosion, &PhysicsFractureTestSystem::smallExplosionTest>();
        bindToEvent<mediumExplosion, &PhysicsFractureTestSystem::mediumExplosionTest>();
        bindToEvent<largeExplosion, &PhysicsFractureTestSystem::largeExplosionTest>();

        bindToEvent<explosion, &PhysicsFractureTestSystem::prematureExplosion>();
        bindToEvent<QHULL, &PhysicsFractureTestSystem::quickHullStep>();
        bindToEvent<AddRigidbody, &PhysicsFractureTestSystem::AddRigidbodyToQuickhulls>();

        #pragma endregion

        //compositeColliderTest();
        //fractureTest();
        //numericalRobustnessTest();
        //simpleMinecraftHouse();
        //explosionTest();
        //fractureVideoScene();
        quickhullTestScene();
        /*meshSplittingTest(planeH, cubeH
            , cylinderH, complexH, textureH);*/

        auto lightshader = rendering::ShaderCache::create_shader("light", "assets://shaders/light.shs"_view);
        directionalLightMH = rendering::MaterialCache::create_material("directional light", lightshader);
        directionalLightMH.set_param("color", math::color(1, 1, 0.8f));
        directionalLightMH.set_param("intensity", 1.f);

        {
            auto sun = createEntity();
            sun.add_components<rendering::mesh_renderable>(mesh_filter(directionalLightH.get_mesh()), rendering::mesh_renderer(directionalLightMH));
            sun.add_component<rendering::light>(rendering::light::directional(math::color(1, 1, 0.8f), 10.f));
            sun.add_components<transform>(position(10, 10, 10), rotation::lookat(math::vec3(1, 1, -1), math::vec3::zero), scale());
        }

        //fractureVideoScene();

        createProcess<&PhysicsFractureTestSystem::colliderDraw>("Physics",0.02f);
        createProcess<&PhysicsFractureTestSystem::explodeAThing>("Physics");

        Fracturer::registry = m_ecs;
    }

    void PhysicsFractureTestSystem::colliderDraw(time::span dt)
    {
        drawPhysicsColliders();

        auto query = createQuery<ObjectToFollow>();
        query.queryEntities();

        for (auto ent : query)
        {
            auto objToFollow = ent.read_component<ObjectToFollow>();

            auto [posH,rotH,scaleH] = ent.get_component_handles<transform>();
            auto [posH2, rotH2, scaleH2] = objToFollow.ent.get_component_handles<transform>();

            posH.write(posH2.read());
            rotH.write(rotH2.read());
            scaleH.write(scaleH2.read());
        }

    }

    void PhysicsFractureTestSystem::meshSplittingTest(rendering::model_handle planeH, rendering::model_handle cubeH
        , rendering::model_handle cylinderH, rendering::model_handle complexH, rendering::material_handle TextureH)
    {
       

        physics::cube_collider_params cubeParams;
        cubeParams.breadth = 2.0f;
        cubeParams.width = 2.0f;
        cubeParams.height = 2.0f;

        //-------------------------------------------------------------------------------------------------------------------------------//
                                                     //CUBE TEST 
        //-------------------------------------------------------------------------------------------------------------------------------//

        ecs::entity_handle cubeSplit2;
        {
            auto splitter = m_ecs->createEntity();
            cubeSplit2 = splitter;

            auto entPhyHande = splitter.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;

            physicsComponent2.AddBox(cubeParams);

            entPhyHande.write(physicsComponent2);

            splitter.add_components<rendering::mesh_renderable>(mesh_filter(planeH.get_mesh()), rendering::mesh_renderer(TextureH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(splitter);
            positionH.write(math::vec3(37, 1.0f, 10.0f));
            scaleH.write(math::vec3(2.0f));

            auto rotation = rotationH.read();

            rotation *= math::angleAxis(math::deg2rad(-60.0f), math::vec3(1, 0, 0));

            splitter.write_component(rotation);

        }

        //Cube split plane
        ecs::entity_handle cubeSplit;
        {
             auto splitter = m_ecs->createEntity();
             cubeSplit = splitter;

             auto entPhyHande = splitter.add_component<physics::physicsComponent>();

             physics::physicsComponent physicsComponent2;

             physicsComponent2.AddBox(cubeParams);

             entPhyHande.write(physicsComponent2);

             splitter.add_components<rendering::mesh_renderable>(mesh_filter(planeH.get_mesh()), rendering::mesh_renderer(TextureH));

             auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(splitter);
             positionH.write(math::vec3(37, 1.5f, 10.0f));
             scaleH.write(math::vec3(2.0f));

             auto rotation = rotationH.read();

             //rotation *= math::angleAxis(math::deg2rad(60.0f), math::vec3(1, 0, 0));

             splitter.write_component(rotation);

        }

        //Cube 
        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();
            //ent.add_component<addRB>();

            physics::physicsComponent physicsComponent2;


            physicsComponent2.AddBox(cubeParams);

            entPhyHande.write(physicsComponent2);

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(TextureH));
            //renderableHandle.write({ cubeH,TextureH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(37, 1.5f, 10.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto id = idHandle.read();
            id.id = "AABBRbStable";
            idHandle.write(id);

            auto finderH = ent.add_component<physics::MeshSplitter>();

            auto finder = finderH.read();
            finder.splitTester.push_back(cubeSplit);
            finder.splitTester.push_back(cubeSplit2);
            finder.InitializePolygons(ent);
            finderH.write(finder);

        }

        //-------------------------------------------------------------------------------------------------------------------------------//
                                                    //CYLINDER TEST 
        //-------------------------------------------------------------------------------------------------------------------------------//
        
        //Split plane
        ecs::entity_handle cylinderSplit;
        {
            cylinderSplit = m_ecs->createEntity();
           
            cylinderSplit.add_components<rendering::mesh_renderable>(mesh_filter(planeH.get_mesh()), rendering::mesh_renderer(TextureH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(cylinderSplit);
            positionH.write(math::vec3(37, 1.3f, 15.0f));
            scaleH.write(math::vec3(2.0f));

        }

        //Cylinder
        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent;

            physicsComponent.AddBox(cubeParams);
            entPhyHande.write(physicsComponent);

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cylinderH.get_mesh()), rendering::mesh_renderer(TextureH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(37, 1.5f, 15.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto id = idHandle.read();
            id.id = "AABBRbStable";
            idHandle.write(id);


            auto finderH = ent.add_component<physics::MeshSplitter>();

            auto finder = finderH.read();
            finder.splitTester.push_back( cylinderSplit);
            finder.InitializePolygons(ent);
            finderH.write(finder);
        }

        //-------------------------------------------------------------------------------------------------------------------------------//
                                                   //CONCAVE TEST 
        //-------------------------------------------------------------------------------------------------------------------------------//


        //Complex Mesh
        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;


            physicsComponent2.AddBox(cubeParams);

            entPhyHande.write(physicsComponent2);

            ent.add_components<rendering::mesh_renderable>(mesh_filter(concaveTestObject.get_mesh()), rendering::mesh_renderer(TextureH));


            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(37, 2.5f, 22.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto id = idHandle.read();
            id.id = "AABBRbStable";
            idHandle.write(id);

            auto finderH = ent.add_component<physics::MeshSplitter>();

            auto finder = finderH.read();
            //finder.splitTester.push_back(cylinderSplit);
            finder.InitializePolygons(ent);
            finderH.write(finder);
            log::debug("test");

            log::debug("finder size {} " , finder.meshPolygons.size());

           

        }
    }

    void PhysicsFractureTestSystem::collisionResolutionTest(rendering::model_handle cubeH, rendering::material_handle wireframeH)
    {
        physics::cube_collider_params cubeParams;
        cubeParams.breadth = 1.0f;
        cubeParams.width = 1.0f;
        cubeParams.height = 1.0f;

        physics::cube_collider_params staticBlockParams;
        staticBlockParams.breadth = 2.5f;
        staticBlockParams.width = 2.5f;
        staticBlockParams.height = 1.0f;

        float testPos = 10.0f;
        //----------- Static Block To AABB Body Stability Test ------------//
        {
            auto ent = m_ecs->createEntity();

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 15.0f));
            scaleH.write(math::vec3(2.5f, 1.0f, 2.5f));

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

        }

        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;


            physicsComponent2.AddBox(staticBlockParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 15.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto idComp = idHandle.read();
            idComp.id = "AABBStaticStable";
            idHandle.write(idComp);

        }

        {
            staticToAABBEntLinear = m_ecs->createEntity();
            
            auto entPhyHande = staticToAABBEntLinear.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            


            physicsComponent2.AddBox(cubeParams);
            entPhyHande.write(physicsComponent2);

            //auto crb = m_ecs->createComponent<physics::rigidbody>(staticToAABBEnt);
            //auto rbHandle = staticToAABBEnt.add_component<physics::rigidbody>();

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(staticToAABBEnt);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(staticToAABBEntLinear);
            positionH.write(math::vec3(testPos, -0.0f, 15.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(staticToAABBEntLinear);
            auto id = idHandle.read();
            id.id = "AABBRbStable";
            idHandle.write(id);
        }

        //{
        //    auto ent = m_ecs->createEntity();
        //    //physicsUnitTestCD.push_back(staticToAABBEntLinear);
        //    auto entPhyHande = ent.add_component<physics::physicsComponent>();

        //    physics::physicsComponent physicsComponent2;
        //    


        //    physicsComponent2.AddBox(cubeParams);
        //    entPhyHande.write(physicsComponent2);

        //    auto crb = m_ecs->createComponent<physics::rigidbody>(ent);
        //    auto rbHandle = ent.add_component<physics::rigidbody>();

        //    //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(staticToAABBEnt);
        //    //renderableHandle.write({ cubeH), wireframeH });

        //    auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
        //    positionH.write(math::vec3(testPos+0.5f, -1.0f, 15.0f));
        //    scaleH.write(math::vec3(1.0f));

        //    auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
        //    auto id = idHandle.read();
        //    id.id = "AABBRbStable";
        //    idHandle.write(id);
        //}

        //----------- Static Block To AABB Body Rotation Test------------//

        {
            auto ent = m_ecs->createEntity();

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 8.0f));
            scaleH.write(math::vec3(2.5f, 1.0f, 2.5f));

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));
        }

        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            
            physicsComponent2.AddBox(staticBlockParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 8.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto idComp = idHandle.read();
            idComp.id = "AABBStatic";
            idHandle.write(idComp);


        }

        {
            staticToAABBEntRotation = m_ecs->createEntity();
           
            auto entPhyHande = staticToAABBEntRotation.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            

            physicsComponent2.AddBox(cubeParams);
            entPhyHande.write(physicsComponent2);

            //auto crb = m_ecs->createComponent<physics::rigidbody>(staticToAABBEnt);
            //auto rbHandle = staticToAABBEnt.add_component<physics::rigidbody>();

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(staticToAABBEnt);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(staticToAABBEntRotation);
            positionH.write(math::vec3(testPos - 2.7f, -0.0f, 8.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(staticToAABBEntRotation);
            auto id = idHandle.read();
            id.id = "AABBRb";
            idHandle.write(id);
        }

        //----------- Static Block To Rotated Body ------------//

        {
            auto ent = m_ecs->createEntity();

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 2.0f));
            scaleH.write(math::vec3(2.5f, 1.0f, 2.5f));

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));
        }

        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            



            physicsComponent2.AddBox(staticBlockParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 2.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto id = idHandle.read();
            id.id = "OBBStatic";
            idHandle.write(id);
        }

        {
            staticToOBBEnt = m_ecs->createEntity();

            auto entPhyHande = staticToOBBEnt.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);


            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(staticToOBBEnt);
            //renderableHandle.write({ cubeH, wireframeH });
            staticToOBBEnt.add_components<rendering::mesh_renderable>
                (mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(staticToOBBEnt);
            positionH.write(math::vec3(testPos, -0.0f, 2.0f));
            scaleH.write(math::vec3(1.0f));

            auto rot = rotationH.read();
            /*rot *= math::angleAxis(math::radians(90.f), math::vec3(0, 0, 1));
            rot *= math::angleAxis(math::radians(40.f), math::vec3(1, 0, 0));
            rot *= math::angleAxis(math::radians(42.f), math::vec3(0, 1, 0));*/
            //rot *= math::angleAxis(45.f, math::vec3(0, 1, 0));
            rotationH.write(rot);

            auto idHandle = m_ecs->createComponent<physics::identifier>(staticToOBBEnt);
            auto id = idHandle.read();
            id.id = "NON_STATIC";
            idHandle.write(id);


            //log::debug("rb.angularVelocity {}", rb.angularVelocity);

        }

        //----------- Static Block To Rotated Body Edge ------------//

        {
            auto ent = m_ecs->createEntity();

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, -4.0f));
            scaleH.write(math::vec3(2.5f, 1.0f, 2.5f));

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));
        }

        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            
            physicsComponent2.AddBox(staticBlockParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, -4.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto id = idHandle.read();
            id.id = "OBBFoundationStaticEdge";
            idHandle.write(id);
        }

        {
            staticToEdgeEnt = m_ecs->createEntity();

            auto entPhyHande = staticToEdgeEnt.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            
            physicsComponent2.AddBox(cubeParams);
            entPhyHande.write(physicsComponent2);


            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(staticToEdgeEnt);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(staticToEdgeEnt);
            positionH.write(math::vec3(testPos + 3.0f, 2.0f, -4.0f));
            scaleH.write(math::vec3(1.0f));

            auto rot = rotationH.read();
            rot *= math::angleAxis(45.f, math::vec3(1, 0, 0));
            rot *= math::angleAxis(45.f, math::vec3(0, 1, 0));
            rotationH.write(rot);

            auto idHandle = m_ecs->createComponent<physics::identifier>(staticToEdgeEnt);
            auto id = idHandle.read();
            id.id = "OBBRb";
            idHandle.write(id);
            //log::debug("rb.angularVelocity {}", rb.angularVelocity);

        }
    }

    void PhysicsFractureTestSystem::fractureVideoScene()
    {
        //create floors
        //concreteH

        CreateElongatedFloor(math::vec3(20.0, 0.5f, -14.0f), math::quat(), math::vec3(15, 1, 20), tileH);

        CreateElongatedFloor(math::vec3(20.0, 0.5f, 5.0f), math::quat(), math::vec3(15, 1, 20), tileH);

        CreateElongatedFloor(math::vec3(20.0, 3.0f, 22.0f)
            , math::quat(math::angleAxis(math::deg2rad(-20.0f), math::vec3(1, 0, 0))),
            math::vec3(15, 1, 20), concreteH);

        CreateElongatedFloor(math::vec3(20.0, 6.0f, 40.5f), math::quat(), math::vec3(15, 1, 20), concreteH);

        CreateElongatedFloor(math::vec3(20.0, 6.0f, 58.5f), math::quat(), math::vec3(15, 1, 20), tileH);

        CreateElongatedFloor(math::vec3(20.0, 6.0f, 78.5f), math::quat(), math::vec3(15, 1, 20), tileH);

        CreateSplitTestBox(physics::cube_collider_params(1.0f, 1.0f, 1.0f), math::vec3(20.0, 1.5f, -14.0f),
            math::quat(), textureH, true, false, math::vec3(),10.0f,1.0f);

        CreateSplitTestBox(physics::cube_collider_params(1.0f, 1.0f, 1.0f), math::vec3(20.0, 1.5f, -6.0f),
            math::quat(), textureH, true, false, math::vec3(), 50.0f, 2.0f);

        CreateSplitTestBox(physics::cube_collider_params(1.0f, 1.0f, 1.0f), math::vec3(20.0, 1.5f, 4.0f),
            math::quat(), textureH, true, false, math::vec3(),80.0f,3.0f);

        float initialTime = 6.0f;
        for (size_t i = 2; i < 7; i++)
        {
            bool shouldExplode = i % 2 == 0;

            CreateSplitTestBox(physics::cube_collider_params(1.0f, 1.0f, 1.0f), math::vec3(20.0, i - 0.5f, 13.0f),
                math::quat(), textureH, shouldExplode, !shouldExplode, math::vec3(),25.0,initialTime);
        }



        spawnCubeStack(math::vec3(18.5f, 7.0f, 37.0f));
        simpleMinecraftHouse(math::vec3(18.0f,7.0f,80.0f));



        //
        CreateElongatedFloor(math::vec3(20.0, 0.5f, 5.0f), math::quat(), math::vec3(15, 1, 20), concreteH);
    }

    void PhysicsFractureTestSystem::prematureExplosion(explosion* action)
    {
        if (!action->value)
        {
            log::debug("Sudden Boom");

            auto countdownQuery = createQuery<FractureCountdown>();
            countdownQuery.queryEntities();

            for (auto ent : countdownQuery)
            {
                auto fractureCountdownH = ent.get_component_handle<FractureCountdown>();

                auto fractureCountdown = fractureCountdownH.read();
                fractureCountdown.explodeNow = true;
                fractureCountdownH.write(fractureCountdown);
            }
        }
       
    }

    void PhysicsFractureTestSystem::quickhullTestScene()
    {
        math::mat3 elongatedBlockInertia = math::mat3(math::vec3(6.0f, 0, 0), math::vec3(0.0f, 18.0f, 0), math::vec3(0, 0, 6.0f));

        //cube
        createQuickhullTestObject
        (math::vec3(0,5.0f, -0.8f),cubeH, concreteH);

        //cup
        createQuickhullTestObject
        (math::vec3(5.0f, 5.0f, -0.8f), colaH, wireFrameH, elongatedBlockInertia);

        //////hammer
        createQuickhullTestObject
        (math::vec3(10.0f, 5.0f, -0.8f), hammerH, rockTextureH);

        ////suzanne
        createQuickhullTestObject
        (math::vec3(15.0f, 5.0f, -0.8f), suzzaneH, wireFrameH);

        ////ohio teapot
        createQuickhullTestObject
        (math::vec3(20.0f, 5.0f, -0.5f), teapotH, wireFrameH,elongatedBlockInertia);
        

        addStaircase(math::vec3(8, 2, 0));
        addStaircase(math::vec3(8, 1, -1));
        addStaircase(math::vec3(8, 0, -2));
        addStaircase(math::vec3(8, -1, -3.1f));

        addStaircase(math::vec3(8, -2, -5),5.0f);


        for (size_t i = 0; i < registeredColliderColorDraw.size(); i++)
        {
            folowerObjects.push_back(std::vector<ecs::entity_handle>());
        }
    }

    void PhysicsFractureTestSystem::addStaircase(math::vec3 position, float breadthMult)
    {
        physics::cube_collider_params cubeParams;
        cubeParams.breadth = breadthMult;
        cubeParams.width = 27.0f;
        cubeParams.height = 1.0f;

        auto ent = m_ecs->createEntity();

        auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
        positionH.write(position);

        //ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(textureH));

        auto entPhyHande = ent.add_component<physics::physicsComponent>();
        physics::physicsComponent physicsComponent2;
        physicsComponent2.AddBox(cubeParams);
        entPhyHande.write(physicsComponent2);

        auto ent2 = m_ecs->createEntity();
        ent2.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(textureH));

        auto [position2H, rotation2H, scale2H] = m_ecs->createComponents<transform>(ent2);
        position2H.write(position);
        scale2H.write(math::vec3(cubeParams.width, 1.0f, breadthMult));
    }

    void PhysicsFractureTestSystem::createQuickhullTestObject(math::vec3 position, rendering::model_handle cubeH, rendering::material_handle TextureH, math::mat3 inertia )
    {
        physics::cube_collider_params cubeParams;
        cubeParams.breadth = 1.0f;
        cubeParams.width = 1.0f;
        cubeParams.height = 1.0f;

        auto ent = m_ecs->createEntity();

        auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
        positionH.write(position);

        ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(TextureH));

        auto entPhyHande = ent.add_component<physics::physicsComponent>();

        auto rbH = ent.add_component<rigidbody>();
        auto rb = rbH.read();
        rb.localInverseInertiaTensor = inertia;
        rbH.write(rb);

        registeredColliderColorDraw.push_back(ent);
    }

    void PhysicsFractureTestSystem::PopulateFollowerList(ecs::entity_handle physicsEnt, int index)
    {
        app::window window = m_ecs->world.get_component_handle<app::window>().read();

        auto physicsComp = physicsEnt.read_component<physicsComponent>();
        auto collider = std::dynamic_pointer_cast<ConvexCollider>(physicsComp.colliders.at(0));
        auto [posH, rotH, scaleH] = physicsEnt.get_component_handles<transform>();

        auto& currentContainer = folowerObjects.at(index);

        for (auto ent : currentContainer)
        {
            m_ecs->destroyEntity(ent);
        }

        folowerObjects.at(index).clear();

        for (auto face : collider->GetHalfEdgeFaces())
        {
            //populate localVert
            math::vec3 localCentroid = face->centroid;
            std::vector<math::vec3> localVert;

            auto populateVectorLambda = [&localVert](physics::HalfEdgeEdge* edge)
            {
                localVert.push_back(edge->edgePosition);
            };

            face->forEachEdge(populateVectorLambda);

            //initialize mesh
            legion::core::mesh newMesh;

            std::vector<math::vec3> vertices;
            std::vector<uint> indices;
            std::vector<math::vec3> normals;
            std::vector<math::vec2> uvs;

            for (size_t i = 0; i < localVert.size(); i++)
            {
                vertices.push_back(localVert.at(i));
                vertices.push_back(localVert.at((i + 1) % localVert.size()));
                vertices.push_back(localCentroid);
            }

            for (size_t i = 0; i < localVert.size(); i++)
            {
                normals.push_back(math::vec3(face->normal));
            }

            for (size_t i = 0; i < vertices.size(); i++)
            {
                indices.push_back(i);
            }

            for (size_t i = 0; i < vertices.size(); i++)
            {
                uvs.push_back(math::vec2());
            }


            newMesh.vertices = vertices;
            newMesh.indices = indices;
            newMesh.uvs = uvs;
            newMesh.normals = normals;

            newMesh.calculate_tangents(&newMesh);

            sub_mesh newSubMesh;
            newSubMesh.indexCount = newMesh.indices.size();
            newSubMesh.indexOffset = 0;

            newMesh.submeshes.push_back(newSubMesh);

            //creaate modelH
            static int count = 0;
            mesh_handle meshH = core::MeshCache::create_mesh("newMesh" + std::to_string(count), newMesh);
            auto modelH = rendering::ModelCache::create_model(meshH);


            auto newEnt = m_ecs->createEntity();

            rendering::material_handle newMat;
            {
                app::context_guard guard(window);
                auto colorShader = rendering::ShaderCache::create_shader("color" + std::to_string(count), fs::view("assets://shaders/color.shs"));
                newMat = rendering::MaterialCache::create_material("vertex color" + std::to_string(count), colorShader);
                newMat.set_param("color", math::color(math::linearRand(0.25f, 0.7f), math::linearRand(0.25f, 0.7f), math::linearRand(0.25f, 0.7f)));
            }

            mesh_filter meshFilter = mesh_filter(meshH);
            //mesh_filter(cubeH.get_mesh()),
            newEnt.add_components<rendering::mesh_renderable>(meshFilter, rendering::mesh_renderer(newMat));

            
            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(newEnt);
            positionH.write(posH.read());
            count++;

            ObjectToFollow followObj;
            followObj.ent = physicsEnt;
            newEnt.add_component(followObj);

            //newEnt.set_parent(physicsEnt);
            currentContainer.push_back(newEnt);

        }


      
    }



    void PhysicsFractureTestSystem::drawPhysicsColliders()
    {
        static float offset = 0.005f;
        static auto physicsQuery = createQuery< physics::physicsComponent>();
        physicsQuery.queryEntities();

        for (auto entity : physicsQuery)
        {
            auto rotationHandle = entity.get_component_handle<rotation>();
            auto positionHandle = entity.get_component_handle<position>();
            auto scaleHandle = entity.get_component_handle<scale>();
            auto physicsComponentHandle = entity.get_component_handle<physics::physicsComponent>();

            bool hasTransform = rotationHandle && positionHandle && scaleHandle;
            bool hasNecessaryComponentsForPhysicsManifold = hasTransform && physicsComponentHandle;

            if (hasNecessaryComponentsForPhysicsManifold)
            {
                auto rbColor = math::color(0.0, 0.5, 0, 1);
                auto statibBlockColor = math::color(0, 1, 0, 1);

                rotation rot = rotationHandle.read();
                position pos = positionHandle.read();
                scale scale = scaleHandle.read();

                auto usedColor = rbColor;
                bool useDepth = false;

                if (entity.get_component_handle<physics::rigidbody>())
                {
                    usedColor = rbColor;
                    //useDepth = true;
                }


                //assemble the local transform matrix of the entity
                math::mat4 localTransform;
                math::compose(localTransform, scale, rot, pos);

                auto physicsComponent = physicsComponentHandle.read();

                for (auto physCollider : physicsComponent.colliders)
                {
                    
                    //--------------------------------- Draw Collider Outlines ---------------------------------------------//
                    if (!physCollider->shouldBeDrawn) { continue; }
                    //math::vec3 colliderCentroid = pos + math::vec3(localTransform * math::vec4(physCollider->GetLocalCentroid(), 0));
                    //debug::drawLine(colliderCentroid, colliderCentroid + math::vec3(0.0f,0.2f,0.0f), math::colors::cyan, 6.0f,0.0f,true);

                    for (auto face : physCollider->GetHalfEdgeFaces())
                    {

                        //face->forEachEdge(drawFunc);
                        physics::HalfEdgeEdge* initialEdge = face->startEdge;
                        physics::HalfEdgeEdge* currentEdge = face->startEdge;
                        math::vec3 worldNormal = (localTransform * math::vec4(face->normal, 0));
                        math::vec3 faceStart = localTransform * math::vec4(face->centroid, 1);
                        math::vec3 faceEnd = faceStart + worldNormal * 0.1f;

                        //debug::drawLine(faceStart, faceEnd, math::colors::green, 2.0f);

                        if (!currentEdge) { return; }

                        do
                        {
                            physics::HalfEdgeEdge* edgeToExecuteOn = currentEdge;
                            currentEdge = currentEdge->nextEdge;
                            math::vec3 shift = worldNormal * offset;

                            math::vec3 worldStart = (localTransform * math::vec4(edgeToExecuteOn->edgePosition, 1)) ;
                            math::vec3 worldEnd = (localTransform * math::vec4(edgeToExecuteOn->nextEdge->edgePosition, 1)) ;

                            //debug::drawLine(worldStart + shift, worldEnd + shift, usedColor, 2.0f, 0.0f, useDepth);

                            if (auto pairing = edgeToExecuteOn->pairingEdge)
                            {
                                //math::vec3 currentEdgeConnect = worldStart + shift + (worldEnd - worldStart + shift * 2.0f) * 0.25;
                                //math::vec3 currentMeet = worldStart + (worldEnd - worldStart) * 0.25;
                                ////debug::drawLine(currentEdgeConnect, currentMeet, math::colors::red, 5.0f, 0.0f, useDepth);

                                // math::vec3 pairingWorldStart = (localTransform * math::vec4(pairing->edgePosition, 1));
                                //math::vec3 pairinWorldEnd = (localTransform * math::vec4(pairing->nextEdge->edgePosition, 1));

                                //math::vec3 pairingMeet = pairingWorldStart + (pairinWorldEnd - pairingWorldStart) * 0.25;
                                //math::vec3 pairingEdgeConnect = pairingWorldStart + shift +
                                //    (pairinWorldEnd - pairingWorldStart + shift * 2.0f) * 0.25;

                                //debug::drawLine(pairingEdgeConnect, pairingMeet, math::colors::red, 5.0f, 0.0f, useDepth);
                            }

                        } while (initialEdge != currentEdge && currentEdge != nullptr);
                    }
                }

            }
        }
    }

    int step = 0;
    int maxStep = 0;

    void PhysicsFractureTestSystem::quickHullStep(QHULL * action)
    {
        if (!action->value)
        {
            int i = 0;
            for (auto ent : registeredColliderColorDraw)
            {
                //[1] Get transform
                auto [posH,rotH,scaleH] = ent.get_component_handles<transform>();

                math::mat4 transform = math::compose(scaleH.read(), rotH.read(), posH.read());

                //auto 
                auto meshFilter = ent.read_component<mesh_filter>();

                //[1] clear colliders list
                auto physicsComponentH = ent.get_component_handle<physics::physicsComponent>();
                auto physComp = physicsComponentH.read();
                physComp.colliders.clear();
                physComp.ConstructConvexHull(meshFilter, step, transform);
                physicsComponentH.write(physComp);

                //[4] use collider to generate follower objects
                PopulateFollowerList(ent,i);
                i++;
            }

            step++;
            log::debug("PhysicsFractureTestSystem::quickHullStep");
        }

    
      
    }

    void PhysicsFractureTestSystem::AddRigidbodyToQuickhulls(AddRigidbody* action)
    {
        if (!action->value)
        {
            log::debug("Add body");
            for (auto ent : registeredColliderColorDraw)
            {
                ent.add_component<rigidbody>();
            }
        }


    }

    void PhysicsFractureTestSystem::extendedContinuePhysics(extendedPhysicsContinue * action)
    {
        if (action->value)
        {
            physics::PhysicsSystem::IsPaused = false;
        }
        else
        {
            physics::PhysicsSystem::IsPaused = true;
        }

    }

    void PhysicsFractureTestSystem::spawnCubeStack(math::vec3 start)
    {
        std::vector<rendering::material_handle> woodInitialThenStone{ woodTextureH,  rockTextureH,  rockTextureH ,  rockTextureH ,woodTextureH };
        std::vector<rendering::material_handle>Stone{ rockTextureH };
        std::vector<rendering::material_handle>Wood{ woodTextureH };
        std::vector<rendering::material_handle>texture{ textureH };
       
        std::vector<int> lastOnly{ 4 };
        std::vector<int> initialOnly{ 1,2,3 };
        std::vector<int> window{ 2 };
        std::vector<int> ignoreEmpty;

        math::vec3 impactPoint = start + math::vec3(1.5, 0.5f, 0);
        float time = 9.0f;

        float height = start.y;
        createFloor(1, 1, math::vec3(start.x , height, start.z),
            math::vec3(1.0f), cubeH, texture, ignoreEmpty, std::vector<bool>{}, time, impactPoint,true);
        createFloor(2, 1, math::vec3(start.x+1.0f, height, start.z),
            math::vec3(1.0f), cubeH, texture, ignoreEmpty, std::vector<bool>{ true, true}, time, impactPoint,false,100.0f);
        createFloor(1, 1, math::vec3(start.x+3.0f, height, start.z),
            math::vec3(1.0f), cubeH, texture, ignoreEmpty, std::vector<bool>{}, time, impactPoint, true);

        height += 1.0f;
        createFloor(3, 1, math::vec3(start.x + 0.5f, height, start.z),
             math::vec3(1.0f), cubeH, texture, ignoreEmpty, std::vector<bool>{false, true, false}, time,  impactPoint,true, 100.0f);

        height += 1.0f;
        createFloor(2, 1, math::vec3(start.x + 1.0f, height, start.z),
            math::vec3(1.0f), cubeH, texture, ignoreEmpty, std::vector<bool>{}, time, impactPoint, true);

        height += 1.0f;
        createFloor(1, 1, math::vec3(start.x + 1.5f, height, start.z),
            math::vec3(1.0f), cubeH, texture, ignoreEmpty, std::vector<bool>{},  5.0f, impactPoint, true);


    }

    void PhysicsFractureTestSystem::OneTimeContinuePhysics(nextPhysicsTimeStepContinue * action)
    {
        if (!(action->value))
        {
            physics::PhysicsSystem::IsPaused = true;
            physics::PhysicsSystem::oneTimeRunActive = true;
            log::debug(" onNextPhysicsTimeStepRequest");
        }
    }

    void PhysicsFractureTestSystem::compositeColliderTest()
    {
        physics::cube_collider_params cubeParams;
        cubeParams.breadth = 1.0f;
        cubeParams.width = 1.0f;
        cubeParams.height = 1.0f;

        //-------------------------------------------------------------------------------------------------------------------------------//
                                                        //COMPOSITE
        //-------------------------------------------------------------------------------------------------------------------------------//

        ecs::entity_handle composite;
        {
            composite = m_ecs->createEntity();


            auto entPhyHande = composite.add_component<physics::physicsComponent>();
            composite.add_component < physics::rigidbody>();

            physics::physicsComponent physicsComponent2;
            

            physicsComponent2.AddBox(cubeParams);

            physicsComponent2.AddBox(cube_collider_params(1.0f,1.0f,1.0f,math::vec3(1.0f,-1.0f,1.0f)));

            entPhyHande.write(physicsComponent2);

            composite.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(textureH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(composite);
            positionH.write(math::vec3(10, 5.0f, 10.0f));
            scaleH.write(math::vec3(1.0f, 1.0f, 1.0f));

            auto rotation = rotationH.read();

            composite.write_component(rotation);

        }

        ecs::entity_handle floor;
        {
            auto ent = m_ecs->createEntity();
            floor = ent;

            auto entPhyHande = floor.add_component<physics::physicsComponent>();


            physics::physicsComponent physicsComponent2;
            

            physicsComponent2.AddBox(cubeParams);

            entPhyHande.write(physicsComponent2);

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(textureH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(10, 2.0f, 10.0f));
            scaleH.write(math::vec3(5.0f, 1.0f,5.0f));

            auto rotation = rotationH.read();

            ent.write_component(rotation);

        }

    }

    void PhysicsFractureTestSystem::explosionTest()
    {
        CreateElongatedFloor(math::vec3(20.0, 1.0f, 9.8f), math::quat(), math::vec3(20, 1, 20),  concreteH);

        smallExplosionEnt = CreateSplitTestBox(physics::cube_collider_params(1.0f,1.0f,1.0f), math::vec3(12.0, 2.0f, 9.8f),
            math::quat(), textureH, true, false, math::vec3());

        mediumExplosionEnt = CreateSplitTestBox(physics::cube_collider_params(1.0f, 1.0f, 1.0f), math::vec3(20.0, 2.0f, 9.8f),
            math::quat(), textureH, true, false, math::vec3());

        largeExplosionEnt = CreateSplitTestBox(physics::cube_collider_params(1.0f, 1.0f, 1.0f), math::vec3(28.0, 2.0f, 9.8f),
            math::quat(), textureH, true, false, math::vec3());



    }

    void PhysicsFractureTestSystem::spawnEntityOnCameraForward(spawnEntity* action)
    {
        if (!action->value)
        {
            auto cameraQuery = createQuery<rendering::camera>();
            cameraQuery.queryEntities();

            for (auto ent : cameraQuery)
            {
                auto [position, rotation, scale] = ent.get_component_handles<transform>();
                math::mat4 trans = math::compose(scale.read(), rotation.read(), position.read());
                math::vec3 dir = math::normalize(trans * math::vec4(0, 0, 1, 0));

                dir *= 10;

                CreateSplitTestBox(physics::cube_collider_params(1, 1, 1), trans[3],
                    math::quat(), textureH, false, true, dir);

            }

        }
       

    }

    void PhysicsFractureTestSystem::simpleMinecraftHouse(math::vec3 start)
    {
        //Floor
        float time = 13.5f;
        float strength = 500.0f;
        std::vector<rendering::material_handle> woodInitialThenStone{ woodTextureH,  rockTextureH,  rockTextureH ,  rockTextureH ,woodTextureH };
        std::vector<rendering::material_handle>Stone{ rockTextureH };
        std::vector<rendering::material_handle>Wood{ woodTextureH };

        std::vector<int> lastOnly{4 };
        std::vector<int> initialOnly{ 1,2,3 };
        std::vector<int> window{ 2 };
        std::vector<int> ignoreEmpty;

        math::vec3 impactPoint = start + math::vec3(2.5, 3.0f, 2.5);

        float level = start.y;
        createFloor(1, 5, math::vec3(start.x, level, start.z),
            math::vec3(1.0f), cubeH, woodInitialThenStone, ignoreEmpty,
            std::vector<bool>{},0.0f,math::vec3(),false,0.0f,false);

        createFloor(1, 5, math::vec3(start.x + 1.0f, level, start.z),
            math::vec3(1.0f), cubeH, Stone, ignoreEmpty,
            std::vector<bool>{}, 0.0f, math::vec3(), false, 0.0f, false);

        createFloor(1, 5, math::vec3(start.x + 2.0f, level, start.z),
            math::vec3(1.0f), cubeH, Stone, ignoreEmpty,
            std::vector<bool>{});

        createFloor(1, 5, math::vec3(start.x + 3.0f, level, start.z),
            math::vec3(1.0f), cubeH, Stone, ignoreEmpty, std::vector<bool>{});
        createFloor(1, 5, math::vec3(start.x + 4.0f, level, start.z),
            math::vec3(1.0f), cubeH, woodInitialThenStone, ignoreEmpty, std::vector<bool>{});
       
        level += 1.0f;
        //first stack
        createFloor(1, 5, math::vec3(start.x + 0.0f, level, start.z),
            math::vec3(1.0f), cubeH, woodInitialThenStone, ignoreEmpty,
            std::vector<bool>{}, 0.0f, math::vec3(), false, 0.0f, false);

        createFloor(1, 5, math::vec3(start.x + 1.0f, level, start.z),
            math::vec3(1.0f), cubeH, Stone, initialOnly,
            std::vector<bool>{}, 0.0f, math::vec3(), false, 0.0f, false);

        createFloor(1, 1, math::vec3(start.x + 2.0f, level, start.z + 4.0f),
            math::vec3(1.0f), cubeH, Stone, initialOnly, std::vector<bool>{});

        createFloor(1, 5, math::vec3(start.x + 3.0f, level, start.z),
            math::vec3(1.0f), cubeH, Stone, initialOnly,
            std::vector<bool>{true, false}, time, impactPoint, false, strength);

        createFloor(1, 5, math::vec3(start.x + 4.0f, level, start.z),
            math::vec3(1.0f), cubeH, woodInitialThenStone, ignoreEmpty,
            std::vector<bool>{true, true, true, true, true}, time, impactPoint, false, strength);

        //window stack
        level += 1.0f;
        createFloor(1, 5, math::vec3(start.x, level, start.z),
            math::vec3(1.0f), cubeH, woodInitialThenStone, window,
            std::vector<bool>{}, 0.0f, math::vec3(), false, 0.0f, false);

        createFloor(1, 5, math::vec3(start.x + 1.0f, level, start.z),
            math::vec3(1.0f), cubeH, Stone, initialOnly,
            std::vector<bool>{}, 0.0f, math::vec3(), false, 0.0f, false);

        createFloor(1,  1, math::vec3(start.x + 2.0f, level, start.z + 4.0f),
            math::vec3(1.0f), cubeH, Stone, initialOnly, std::vector<bool>{});

        createFloor(1, 5, math::vec3(start.x + 3.0f, level, start.z),
            math::vec3(1.0f), cubeH, Stone, initialOnly, std::vector<bool>{true, true}, time, impactPoint, false, strength);
        createFloor(1, 5, math::vec3(start.x + 4.0f, level, start.z),
            math::vec3(1.0f), cubeH, woodInitialThenStone, window,
            std::vector<bool>{true, true, true, true, true}, time, impactPoint, false, strength);

        level += 1.0f;
        //third stack
        createFloor(1, 5, math::vec3(start.x, level, start.z),
            math::vec3(1.0f), cubeH, woodInitialThenStone, ignoreEmpty,
            std::vector<bool>{}, 0.0f, math::vec3(), false, 0.0f, false);

        createFloor(1, 5, math::vec3(start.x + 1.0f, level, start.z),
            math::vec3(1.0f), cubeH, Stone, initialOnly,
            std::vector<bool>{}, 0.0f, math::vec3(), false, 0.0f, false);

        createFloor(1, 5, math::vec3(start.x + 2.0f, level, start.z),
            math::vec3(1.0f), cubeH, Stone, initialOnly, std::vector<bool>{true,false}, time, impactPoint, false, strength);
        createFloor(1, 5, math::vec3(start.x + 3.0f, level, start.z),
            math::vec3(1.0f), cubeH, Stone, initialOnly, std::vector<bool>{true, true, true, true, true}, time, impactPoint, false, strength);
        createFloor(1, 5, math::vec3(start.x + 4.0f, level, start.z),
            math::vec3(1.0f), cubeH, woodInitialThenStone, ignoreEmpty, std::vector<bool>{true, true, true, true, true}, time, impactPoint, false, strength);

        level += 1.0f;
        createFloor(1, 5, math::vec3(start.x, level, start.z),
            math::vec3(1.0f), cubeH, Wood, ignoreEmpty,
            std::vector<bool>{}, 0.0f, math::vec3(), false, 0.0f, false);

        createFloor(1, 5, math::vec3(start.x + 1.0f, level, start.z),
            math::vec3(1.0f), cubeH, Wood, ignoreEmpty,
            std::vector<bool>{}, 0.0f, math::vec3(), false, 0.0f, false);

        createFloor(1, 5, math::vec3(start.x + 2.0f, level, start.z),
            math::vec3(1.0f), cubeH, Wood, ignoreEmpty, std::vector<bool>{}, time, impactPoint, false, strength);
        createFloor(1, 5, math::vec3(start.x + 3.0f, level, start.z),
            math::vec3(1.0f), cubeH, Wood, ignoreEmpty, std::vector<bool>{true, true, true, true, true}, time, impactPoint, false, strength);
        createFloor(1, 5, math::vec3(start.x + 4.0f, level, start.z),
            math::vec3(1.0f), cubeH, Wood, ignoreEmpty, std::vector<bool>{true, true, true, true, true}, time, impactPoint, false, strength);

        PhysicsSystem::IsPaused = false;
        //CreateElongatedFloor(math::vec3(5.0, 1.0f, 9.8f), math::quat(), math::vec3(20, 1, 20));

    }

    void PhysicsFractureTestSystem::numericalRobustnessTest()
    {
        physics::cube_collider_params cubeParams;
        cubeParams.breadth = 1.0f;
        cubeParams.width = 1.0f;
        cubeParams.height = 1.0f;


        //Position Test
        ecs::entity_handle wall;
        {
            wall = m_ecs->createEntity();
            auto entPhyHande = wall.add_component<physics::physicsComponent>();
            wall.add_component<rigidbody>();
            //wall.add_component<physics::Fracturer>();

            physics::physicsComponent physicsComponent2;


            physicsComponent2.AddBox(cubeParams);

            entPhyHande.write(physicsComponent2);

            wall.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(textureH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(wall);
            positionH.write(math::vec3(0, 12.0f, 10.0f));
            scaleH.write(math::vec3(1.0f, 1.0f, 1.0f));

            auto rotation = rotationH.read();
            rotation *= math::angleAxis(math::deg2rad(30.0f), math::vec3(1, 0, 0));
            rotation *= math::angleAxis(math::deg2rad(50.0f), math::vec3(0, 1, 0));
            wall.write_component(rotation);

            auto splitterH = wall.add_component<physics::MeshSplitter>();
            auto splitter = splitterH.read();
            splitter.InitializePolygons(wall);
            splitterH.write(splitter);


            auto idH = wall.add_component<physics::identifier>();
            auto id = idH.read();
            id.id = "WALL";
            idH.write(id);
        }

        ecs::entity_handle wall2;
        {
            wall2 = m_ecs->createEntity();


            auto entPhyHande = wall2.add_component<physics::physicsComponent>();
            wall2.add_component<rigidbody>();
            //wall2.add_component<physics::Fracturer>();

            physics::physicsComponent physicsComponent2;


            physicsComponent2.AddBox(cubeParams);

            entPhyHande.write(physicsComponent2);

            wall2.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(textureH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(wall2);
            positionH.write(math::vec3(50.0f, 12.0f, 10.0f));
            scaleH.write(math::vec3(1.0f, 1.0f, 1.0f));

            auto rotation = rotationH.read();
            rotation *= math::angleAxis(math::deg2rad(30.0f), math::vec3(1, 0, 0));
            rotation *= math::angleAxis(math::deg2rad(50.0f), math::vec3(0, 1, 0));
            wall2.write_component(rotation);

            auto splitterH = wall2.add_component<physics::MeshSplitter>();
            auto splitter = splitterH.read();
            splitter.InitializePolygons(wall2);
            splitterH.write(splitter);


            //auto idH = wall2.add_component<physics::identifier>();
            //auto id = idH.read();
            //id.id = "WALL";
            //idH.write(id);
        }

        ecs::entity_handle wall3;
        {
            wall3 = m_ecs->createEntity();


            auto entPhyHande = wall3.add_component<physics::physicsComponent>();
            wall3.add_component<rigidbody>();
            //wall2.add_component<physics::Fracturer>();

            physics::physicsComponent physicsComponent2;


            physicsComponent2.AddBox(cubeParams);

            entPhyHande.write(physicsComponent2);

            wall3.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(textureH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(wall3);
            positionH.write(math::vec3(200.0f, 12.0f, 10.0f));
            scaleH.write(math::vec3(1.0f, 1.0f, 1.0f));

            auto rotation = rotationH.read();
            rotation *= math::angleAxis(math::deg2rad(30.0f), math::vec3(1, 0, 0));
            rotation *= math::angleAxis(math::deg2rad(50.0f), math::vec3(0, 1, 0));
            wall3.write_component(rotation);

            auto splitterH = wall3.add_component<physics::MeshSplitter>();
            auto splitter = splitterH.read();
            splitter.InitializePolygons(wall2);
            splitterH.write(splitter);


            //auto idH = wall2.add_component<physics::identifier>();
            //auto id = idH.read();
            //id.id = "WALL";
            //idH.write(id);
        }

        CreateElongatedFloor(math::vec3(0.0f, 1.0f, 10.0f), math::quat(), math::vec3(5.0f, 1.0f, 5.0f),woodTextureH);

        rotation rot = math::angleAxis(math::deg2rad(90.0f), math::vec3(1, 0, 0));
        CreateElongatedFloor(math::vec3(0.0f, 2.0f, 7.0f), rot, math::vec3(5.0f, 1.0f, 5.0f), woodTextureH);

        ////////////////////////////////////////////////
        CreateElongatedFloor(math::vec3(50.0f, 1.0f, 10.0f), math::quat(), math::vec3(5.0f, 1.0f, 5.0f), woodTextureH);
        CreateElongatedFloor(math::vec3(50.0f, 2.0f, 7.0f), rot, math::vec3(5.0f, 1.0f, 5.0f), woodTextureH);

        /////////////////////////////////////////////////////
        CreateElongatedFloor(math::vec3(200.0f, 1.0f, 10.0f), math::quat(), math::vec3(5.0f, 1.0f, 5.0f), woodTextureH);
        CreateElongatedFloor(math::vec3(200.0f, 2.0f, 7.0f), rot, math::vec3(5.0f, 1.0f, 5.0f), woodTextureH);

        CreateElongatedFloor(math::vec3(-50.0f, 0.0f, -50.0f), math::quat(), math::vec3(90.0f, 5.0f, 90.0f), woodTextureH);

        ///////// small,medium,big
        CreateSplitTestBox(physics::cube_collider_params(1.0f, 1.0f, 1.0f), math::vec3(-50.0f, 15.0f, -50.0f),
            math::quat(),textureH, false);

        CreateSplitTestBox(physics::cube_collider_params(1.0f, 1.0f, 1.0f), math::vec3(-35.0f, 15.0f, -35.0f),
            math::quat(), textureH, false);

        CreateSplitTestBox(physics::cube_collider_params(1.0f, 1.0f, 1.0f), math::vec3(-15.0f, 15.0f, -15.0f),
            math::quat(), textureH, false);





    }

    void PhysicsFractureTestSystem::fractureTest()
    {
        physics::cube_collider_params cubeParams;
        cubeParams.breadth = 1.0f;
        cubeParams.width = 1.0f;
        cubeParams.height = 1.0f;


        physics::cube_collider_params scaledCubeParams(-5.0f,5.0f,1.0f);
        //-------------------------------------------------------------------------------------------------------------------------------//
                                                       //WALL
        //-------------------------------------------------------------------------------------------------------------------------------//

        /*CreateSplitTestBox(cubeParams, math::vec3(0.0f, 2.0f, 9.8f),
            math::quat(), true);

        CreateSplitTestBox(cubeParams, math::vec3(0.0f, 3.0f, 9.8f),
            math::quat(), true);*/

 

        //-------------------------------------------------------------------------------------------------------------------------------//
                                                     //THROWN OBJECT TEST 
        //-------------------------------------------------------------------------------------------------------------------------------//

        ecs::entity_handle block;
        {
            block = m_ecs->createEntity();

            auto entPhyHande = block.add_component<physics::physicsComponent>();
            auto rbH = block.add_component<physics::rigidbody>();

            auto rb = rbH.read();
            rb.velocity = math::vec3(6, 0, 0);
            rb.setMass(3.0f);
            rbH.write(rb);

            physics::physicsComponent physicsComponent2;
            

            physicsComponent2.AddBox(cubeParams);

            entPhyHande.write(physicsComponent2);

            block.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(textureH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(block);
            positionH.write(math::vec3(-3.0f, 3.0f, 9.8f));
            scaleH.write(math::vec3(1.0f, 1.0f, 1.0f));

            auto rotation = rotationH.read();
            rotation *= math::angleAxis(math::deg2rad(45.0f), math::vec3(0, 1, 0));
            rotation *= math::angleAxis(math::deg2rad(-60.0f), math::vec3(1, 0, 0));
            block.write_component(rotation);

           

        }

        CreateElongatedFloor(math::vec3(0, 1.0f, 9.8f), math::quat(), math::vec3(5,1,5), concreteH);
    }

    void PhysicsFractureTestSystem::CreateElongatedFloor(math::vec3 position, math::quat rot, math::vec3 scale,rendering::material_handle mat
        ,bool hasCollider )
    {
        if (hasCollider)
        {
            cube_collider_params scaledCubeParams(scale.x, scale.z, scale.y);
            ecs::entity_handle floor5;
            {
                floor5 = m_ecs->createEntity();

                auto entPhyHande = floor5.add_component<physics::physicsComponent>();

                physics::physicsComponent physicsComponent2;
                physicsComponent2.AddBox(scaledCubeParams);
                entPhyHande.write(physicsComponent2);

                //floor.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(woodTextureH));

                auto idH = floor5.add_component<physics::identifier>();
                auto id = idH.read();
                id.id = "floor";
                idH.write(id);


                auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(floor5);
                positionH.write(position);
                rotationH.write(rot);
                scaleH.write(math::vec3(1.0f, 1.0f, 1.0f));
            }

        }
      
        ecs::entity_handle floor6;
        {
            floor6 = m_ecs->createEntity();

            floor6.add_components<rendering::mesh_renderable>
                (mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(mat));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(floor6);
            positionH.write(position);
            rotationH.write(rot);
            scaleH.write(scale);

        }
    }

    void PhysicsFractureTestSystem::OnSplit(physics_split_test* action)
    {
        static ecs::EntityQuery halfEdgeQuery = createQuery<physics::MeshSplitter>();

        if (action->value)
        {
            halfEdgeQuery.queryEntities();

            for (auto entity : halfEdgeQuery)
            {
                auto edgeFinderH = entity.get_component_handle<physics::MeshSplitter>();
                auto edgeFinder = edgeFinderH.read();
                edgeFinder.TestSplit();
                edgeFinderH.write(edgeFinder);
                edgeFinder.DestroyTestSplitter(m_ecs);

            }
        }
    }

    ecs::entity_handle PhysicsFractureTestSystem::CreateSplitTestBox(physics::cube_collider_params cubeParams,math::vec3 position,
        math::quat rotation, rendering::material_handle mat, bool isFracturable,bool hasRigidbody
        ,math::vec3 velocity,float explosionStrength,float explosionTime, math::vec3 impactPoint, bool hasCollider)
    {
        auto ent = m_ecs->createEntity();
  

        if (hasRigidbody)
        {
           auto rbH = ent.add_component<rigidbody>();
           auto rb = rbH.read();
           rb.velocity = velocity;
           rbH.write(rb);
        }

        if (isFracturable)
        {
            ent.add_component<physics::Fracturer>();
            auto FractureCountdownH = ent.add_component<physics::FractureCountdown>();
            auto fractureCountdown = FractureCountdownH.read();
            fractureCountdown.explosionPoint = static_cast<int>(impactPoint.x) == -69 ?  position : impactPoint;
            fractureCountdown.fractureStrength = explosionStrength;
            fractureCountdown.fractureTime = explosionTime;

            FractureCountdownH.write(fractureCountdown);
        }


        physics::physicsComponent physicsComponent2;

        if (hasCollider)
        {
            auto entPhyHande = ent.add_component<physics::physicsComponent>();
            physicsComponent2.AddBox(cubeParams);
            entPhyHande.write(physicsComponent2);
        }


   

        ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(mat));

        auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
        positionH.write(position);
        scaleH.write(math::vec3(1.0f, 1.0f, 1.0f));
        rotationH.write(rotation);

        if (isFracturable)
        {
            auto splitterH = ent.add_component<physics::MeshSplitter>();
            auto splitter = splitterH.read();
            splitter.InitializePolygons(ent);
            splitterH.write(splitter);
        }

        return  ent;
    }

    void PhysicsFractureTestSystem::createFloor(int xCount, int yCount, math::vec3 start,
        math::vec3 offset, rendering::model_handle cubeH,std::vector< rendering::material_handle> materials,
        std::vector<int> ignoreJ, std::vector<bool> shouldFracture, float fractureTime ,
        math::vec3 impactPoint ,bool hasRigidbodies, float strength, bool hasCollider)
    {
        int k = 0;
        for (size_t i = 0; i < xCount; i++)
        {
            for (size_t j = 0; j < yCount; j++)
            {
               
                bool shouldSkip = false;
                for (auto ignoreCount : ignoreJ)
                {
                    if (j == ignoreCount)
                    {
                        shouldSkip = true;
                    }
                }

                if (shouldSkip) { continue; }

                bool shouldFractureFlag = false;

                if (k < shouldFracture.size())
                {
                    shouldFractureFlag = shouldFracture.at(k);

                    
                }


                auto material_handle = materials.at(  j%materials.size());
                CreateSplitTestBox(physics::cube_collider_params(1.0f, 1.0f, 1.0f), start
                    + math::vec3(offset.x * i, 0, offset.z * j),
                    math::quat(), material_handle, shouldFractureFlag, hasRigidbodies, math::vec3(), strength,fractureTime,impactPoint,hasCollider);
                k++;
            }
        }
    }

    void PhysicsFractureTestSystem::smallExplosionTest(smallExplosion* action)
    {
        if (!action->value)
        {
            *reinterpret_cast<int*>(&m_boom) |= SMALL_BOOM;
        }

    }

    void PhysicsFractureTestSystem::mediumExplosionTest(mediumExplosion* action)
    {
        if (!action->value)
        {
            *reinterpret_cast<int*>(&m_boom) |= MEDIUM_BOOM;
        }
       
    }

    void PhysicsFractureTestSystem::largeExplosionTest(largeExplosion* action)
    {
        if (!action->value)
        {
            *reinterpret_cast<int*>(&m_boom) |= BIG_BOOM;
        }
    
    }

    void PhysicsFractureTestSystem::explodeAThing(time::span)
    {
        if (m_boom & SMALL_BOOM)
        {
            log::debug("smallExplosionTest");
            auto PosH = smallExplosionEnt.get_component_handle<position>();
            auto fracturerH = smallExplosionEnt.get_component_handle<Fracturer>();
            auto fracturer = fracturerH.read();
            FractureParams param(PosH.read(), 10.0f);
            fracturer.ExplodeEntity(smallExplosionEnt, param);
            fracturerH.write(fracturer);

        }

        if (m_boom & MEDIUM_BOOM)
        {
            auto PosH = mediumExplosionEnt.get_component_handle<position>();
            auto fracturerH = mediumExplosionEnt.get_component_handle<Fracturer>();
            auto fracturer = fracturerH.read();
            FractureParams param(PosH.read(), 50.0f);
            fracturer.ExplodeEntity(mediumExplosionEnt, param);
            fracturerH.write(fracturer);
        }

        if (m_boom & BIG_BOOM)
        {
            auto PosH = largeExplosionEnt.get_component_handle<position>();
            auto fracturerH = largeExplosionEnt.get_component_handle<Fracturer>();
            auto fracturer = fracturerH.read();
            FractureParams param(PosH.read(), 200.0f);
            fracturer.ExplodeEntity(largeExplosionEnt, param);
            fracturerH.write(fracturer);
        }

        m_boom = NO_BOOM;
    }
    

}


