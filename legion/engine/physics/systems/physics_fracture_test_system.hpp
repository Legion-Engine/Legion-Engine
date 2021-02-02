#pragma once
#include <core/core.hpp>
#include <rendering/components/renderable.hpp>
#include <physics/cube_collider_params.hpp>
using namespace legion;

struct extendedPhysicsContinue : public app::input_action<extendedPhysicsContinue> {};
struct nextPhysicsTimeStepContinue : public app::input_action<nextPhysicsTimeStepContinue> {};
struct physics_split_test : public app::input_action<physics_split_test> {};
struct spawnEntity : public app::input_action<spawnEntity> {};

struct smallExplosion : public app::input_action<smallExplosion> {};
struct mediumExplosion : public app::input_action<mediumExplosion> {};
struct largeExplosion : public app::input_action<largeExplosion> {};


namespace legion::physics
{
    class PhysicsFractureTestSystem final : public System<PhysicsFractureTestSystem>
    {
    public:

        virtual void setup();

        virtual void colliderDraw(time::span dt);

    private:

        void CreateElongatedFloor(math::vec3 position,math::quat rot, math::vec3 scale);

        ecs::entity_handle CreateSplitTestBox(physics::cube_collider_params cubeParams, math::vec3 position,
            math::quat rotation, rendering::material_handle mat, bool isFracturable, bool hasRigidbody = false
            , math::vec3 velocity = math::vec3(),ecs::entity_handle ent = m_ecs->createEntity());

        void OnSplit(physics_split_test* action);

        void meshSplittingTest(rendering::model_handle planeH, rendering::model_handle cubeH
            , rendering::model_handle cylinderH, rendering::model_handle complexH, rendering::material_handle TextureH);

        void collisionResolutionTest(rendering::model_handle cubeH,
            rendering::material_handle wireframeH);

        void numericalRobustnessTest();

        void extendedContinuePhysics(extendedPhysicsContinue * action);

        void OneTimeContinuePhysics(nextPhysicsTimeStepContinue * action);

        void compositeColliderTest();

        void explosionTest();

        void spawnEntityOnCameraForward(spawnEntity * action);

        void simpleMinecraftHouse();

        void createFloor(int xCount, int yCount, math::vec3 start,
            math::vec3 offset, rendering::model_handle cubeH, std::vector< rendering::material_handle> materials
            , std::vector<int> ignoreJ);

        void smallExplosionTest(smallExplosion*action);
        void mediumExplosionTest(mediumExplosion*action);
        void largeExplosionTest(largeExplosion*action);

        void explodeAThing(time::span);

        enum explosionType : int
        {
            NO_BOOM = 0,
            SMALL_BOOM = 1,
            MEDIUM_BOOM = 2,
            BIG_BOOM = 4,
        };

        explosionType m_boom = explosionType::NO_BOOM;

        void fractureTest();
        rendering::material_handle textureH;
        rendering::material_handle woodTextureH;
        rendering::material_handle rockTextureH;


        rendering::model_handle cubeH;
        rendering::model_handle concaveTestObject;
        rendering::model_handle planeH;
        rendering::model_handle cylinderH;
        rendering::model_handle complexH;

        ecs::entity_handle smallExplosionEnt;
        ecs::entity_handle mediumExplosionEnt;
        ecs::entity_handle largeExplosionEnt;

        ecs::entity_handle staticToAABBEntLinear, staticToAABBEntRotation, staticToOBBEnt, staticToEdgeEnt;

    };
}
