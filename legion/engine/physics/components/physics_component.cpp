
#include <physics/components/physics_component.hpp>
#include <physics/colliders/convexcollider.hpp>
#include <physics/physics_statics.hpp>

namespace legion::physics
{
    void physicsComponent::calculateNewLocalCenterOfMass()
    {
        localCenterOfMass = math::vec3::zero;

        for (auto collider : colliders)
        {
            localCenterOfMass += collider->GetLocalCentroid();
        }

        localCenterOfMass /= static_cast<float>(colliders.size());
    }

    std::shared_ptr<ConvexCollider> physicsComponent::ConstructConvexHull(legion::core::mesh_handle meshHandle, int DEBUG_stepMax,
        math::mat4 DEBUG_transform)
    {
        const auto& vertices = meshHandle.get().second.vertices;
        auto collider = PhysicsStatics::generateConvexHull(vertices);

        if (collider)
        {
            colliders.push_back(collider);

            calculateNewLocalCenterOfMass();
        }
   
        return collider;
    }

    void physicsComponent::ConstructConvexHull(legion::core::mesh_handle meshHandle, ConvexCollider& col)
    {
        col.doStep(meshHandle);
        calculateNewLocalCenterOfMass();
    }

    void physicsComponent::ConstructBox()
    {

        calculateNewLocalCenterOfMass();
    }

    void physicsComponent::AddBox(const cube_collider_params& cubeParams)
    {
        auto cuboidCollider = std::make_shared<ConvexCollider>();

        cuboidCollider->CreateBox(cubeParams);

        colliders.push_back(cuboidCollider);

        calculateNewLocalCenterOfMass();
    }

    void physicsComponent::AddSphere()
    {
        calculateNewLocalCenterOfMass();
    }
}

