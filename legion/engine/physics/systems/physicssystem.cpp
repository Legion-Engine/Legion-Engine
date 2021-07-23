#include <physics/systems/physicssystem.hpp>
#include <physics/broadphasecollisionalgorithms/broadphaseuniformgridnocaching.hpp>

namespace legion::physics
{
    std::unique_ptr<BroadPhaseCollisionAlgorithm> PhysicsSystem::m_broadPhase = nullptr;

    bool PhysicsSystem::IsPaused = true;
    bool PhysicsSystem::oneTimeRunActive = false;


    void PhysicsSystem::setup()
    {
        createProcess<&PhysicsSystem::fixedUpdate>("Physics", m_timeStep);

        manifoldPrecursorQuery = createQuery<position, rotation, scale, physicsComponent>();

        //std::make_unique<BroadphaseUniformGrid>(math::vec3(2,2,2),1);
        //std::make_unique<BroadphaseBruteforce>();
        //std::make_unique<BroadphaseUniformGridNoCaching>(math::vec3(2, 2, 2));

        m_broadPhase = std::make_unique<BroadphaseUniformGridNoCaching>(math::vec3(2, 2, 2));

    }

    void PhysicsSystem::runPhysicsPipeline(
        std::vector<byte>& hasRigidBodies,
        ecs::component_container<rigidbody>& rigidbodies,
        ecs::component_container<physicsComponent>& physComps,
        ecs::component_container<position>& positions,
        ecs::component_container<rotation>& rotations,
        ecs::component_container<scale>& scales,
        float deltaTime)
    {
        OPTICK_EVENT();

        //-------------------------------------------------Broadphase Optimization-----------------------------------------------//

        //get all physics components from the world
        std::vector<physics_manifold_precursor> manifoldPrecursors;
        bulkRetrievePreManifoldData(physComps, positions, rotations, scales, manifoldPrecursors);

        std::vector<std::vector<physics_manifold_precursor>> manifoldPrecursorGrouping;
        //m_optimizeBroadPhase(manifoldPrecursors, manifoldPrecursorGrouping);
        manifoldPrecursorGrouping = m_broadPhase->collectPairs(std::move(manifoldPrecursors));

        //------------------------------------------------------ Narrowphase -----------------------------------------------------//
        std::vector<physics_manifold> manifoldsToSolve;

        {
            OPTICK_EVENT("Narrowphase");

            std::set<std::pair<id_type, id_type>> idPairings;

            size_type totalChecks = 0;
            for (auto& manifoldPrecursor : manifoldPrecursorGrouping)
            {
                if (manifoldPrecursor.size() == 0) continue;
                for (int i = 0; i < manifoldPrecursor.size() - 1; i++)
                {
                    for (int j = i + 1; j < manifoldPrecursor.size(); j++)
                    {
                        totalChecks++;
                        assert(j != manifoldPrecursor.size());

                        physics_manifold_precursor& precursorA = manifoldPrecursor.at(i);
                        physics_manifold_precursor& precursorB = manifoldPrecursor.at(j);

                        //check if we have found this pairing before
                        std::pair<id_type, id_type> precursorPairing = std::make_pair(precursorA.id, precursorB.id);
                        auto foundPairingIter = idPairings.find(precursorPairing);

                        if (foundPairingIter != idPairings.end())
                        {
                            //early out so we dont solve the collision twice
                            continue;
                        }
                        else
                        {
                            idPairings.insert(precursorPairing);
                        }
                       

                        auto& precursorPhyCompA = *precursorA.physicsComp;
                        auto& precursorPhyCompB = *precursorB.physicsComp;

                        auto& precursorRigidbodyA = rigidbodies[precursorA.id];
                        auto& precursorRigidbodyB = rigidbodies[precursorB.id];

                        //only construct a manifold if at least one of these requirement are fulfilled
                        //1. One of the physicsComponents is a trigger and the other one is not
                        //2. One of the physicsComponent's entity has a rigidbody and the other one is not a trigger
                        //3. Both have a rigidbody

                        bool isBetweenTriggerAndNonTrigger =
                            (precursorPhyCompA.isTrigger && !precursorPhyCompB.isTrigger) || (!precursorPhyCompA.isTrigger && precursorPhyCompB.isTrigger);

                        bool isBetweenRigidbodyAndNonTrigger =
                            (hasRigidBodies[precursorA.id] && !precursorPhyCompB.isTrigger) || (hasRigidBodies[precursorB.id] && !precursorPhyCompA.isTrigger);

                        bool isBetween2Rigidbodies = (hasRigidBodies[precursorA.id] && hasRigidBodies[precursorB.id]);


                        if (isBetweenTriggerAndNonTrigger || isBetweenRigidbodyAndNonTrigger || isBetween2Rigidbodies)
                        {
                            constructManifoldsWithPrecursors(rigidbodies, hasRigidBodies, precursorA, precursorB,
                                manifoldsToSolve,
                                hasRigidBodies[precursorA.id] || hasRigidBodies[precursorB.id]
                                , precursorPhyCompA.isTrigger || precursorPhyCompB.isTrigger);
                        }
                    }
                }
            }
            //log::debug("groupings {}", manifoldPrecursorGrouping.size());
            //log::debug("total checks {}", totalChecks);
        }

        //------------------------------------------------ Pre Collision Solve Events --------------------------------------------//


            //explosion event
        auto countdownQuery = createQuery<FractureCountdown>();
        countdownQuery.queryEntities();

        for (auto ent : countdownQuery)
        {
            auto fractureCountdown = ent.read_component<FractureCountdown>();
            fractureCountdown.fractureTime -= 0.02f;
            //log::debug(" fractureCountdown.fractureTime {}", fractureCountdown.fractureTime);

            if (fractureCountdown.explodeNow || fractureCountdown.fractureTime < 0.0f)
            {
                log::debug("Entity is exploding");

                auto fracturerH = ent.get_component_handle<Fracturer>();
                auto fracturer = fracturerH.read();
                log::debug("fractureCountdown.explosionPoint {} ", fractureCountdown.explosionPoint);
                log::debug("fractureCountdown.fractureStrength {} ", fractureCountdown.fractureStrength);
                FractureParams params(fractureCountdown.explosionPoint, fractureCountdown.fractureStrength);

                fracturer.ExplodeEntity(ent, params);

                fracturerH.write(fracturer);

            }

            ent.write_component(fractureCountdown);
        }


        // all manifolds are initially valid

        std::vector<byte> manifoldValidity(manifoldsToSolve.size(), true);

        //TODO we are currently hard coding fracture, this should be an event at some point
        {
            OPTICK_EVENT("Fracture");
            for (size_t i = 0; i < manifoldsToSolve.size(); i++)
            {
                auto& manifold = manifoldsToSolve.at(i);

                auto& entityHandleA = manifold.entityA;
                auto& entityHandleB = manifold.entityB;

                auto fracturerHandleA = entityHandleA.get_component_handle<Fracturer>();
                auto fracturerHandleB = entityHandleB.get_component_handle<Fracturer>();

                bool currentManifoldValidity = manifoldValidity.at(i);

                //log::debug("- A Fracture Check");

                if (fracturerHandleA)
                {
                    auto fracturerA = fracturerHandleA.read();
                    //log::debug(" A is fracturable");
                    fracturerA.HandleFracture(manifold, currentManifoldValidity, true);

                    fracturerHandleA.write(fracturerA);
                }

                //log::debug("- B Fracture Check");

                if (fracturerHandleB)
                {
                    auto fracturerB = fracturerHandleB.read();
                    //log::debug(" B is fracturable");
                    fracturerB.HandleFracture(manifold, currentManifoldValidity, false);

                    fracturerHandleB.write(fracturerB);
                }

                manifoldValidity.at(i) = currentManifoldValidity;
            }
        }
        //-------------------------------------------------- Collision Solver ---------------------------------------------------//
        //for both contact and friction resolution, an iterative algorithm is used.
        //Everytime physics_contact::resolveContactConstraint is called, the rigidbodies in question get closer to the actual
        //"correct" linear and angular velocity (Projected Gauss Seidel). For the sake of simplicity, an arbitrary number is set for the
        //iteration count.

        //the effective mass remains the same for every iteration of the solver. This means that we can precalculate it before
        //we start the solver

        //log::debug("--------------Logging contacts for manifold -------------------");
        {
            OPTICK_EVENT("Resolve collisions");

            initializeManifolds(manifoldsToSolve, manifoldValidity);

            {
                OPTICK_EVENT("Resolve contact constraints");

                //resolve contact constraint
                for (size_t contactIter = 0;
                    contactIter < constants::contactSolverIterationCount; contactIter++)
                {
                    resolveContactConstraint(manifoldsToSolve, manifoldValidity, deltaTime, contactIter);
                }
            }

            {
                OPTICK_EVENT("Resolve friction constraints");

                //resolve friction constraint
                for (size_t frictionIter = 0;
                    frictionIter < constants::frictionSolverIterationCount; frictionIter++)
                {
                    resolveFrictionConstraint(manifoldsToSolve, manifoldValidity);
                }
            }

            {
                OPTICK_EVENT("Converge manifolds");

                //reset convergance identifiers for all colliders
                for (auto& manifold : manifoldsToSolve)
                {
                    manifold.colliderA->converganceIdentifiers.clear();
                    manifold.colliderB->converganceIdentifiers.clear();
                }

                //using the known lambdas of this time step, add it as a convergance identifier
                for (auto& manifold : manifoldsToSolve)
                {
                    for (auto& contact : manifold.contacts)
                    {
                        contact.refCollider->AddConverganceIdentifier(contact);
                    }
                }
            }
        }

    }

    void PhysicsSystem::constructManifoldsWithPrecursors(ecs::component_container<rigidbody>& rigidbodies, std::vector<byte>& hasRigidBodies, physics_manifold_precursor& precursorA, physics_manifold_precursor& precursorB,
        std::vector<physics_manifold>& manifoldsToSolve, bool isRigidbodyInvolved, bool isTriggerInvolved)
    {
        OPTICK_EVENT();
        if (!precursorA.physicsComp || !precursorB.physicsComp) return;
        auto& physicsComponentA = *precursorA.physicsComp;
        auto& physicsComponentB = *precursorB.physicsComp;

        //if (physicsComponentA.colliders.empty() || physicsComponentB.colliders.empty()) return;

        for (auto colliderA : physicsComponentA.colliders)
        {
            for (auto colliderB : physicsComponentB.colliders)
            {
                physics::physics_manifold m;
                constructManifoldWithCollider(rigidbodies, hasRigidBodies, colliderA.get(), colliderB.get(), precursorA, precursorB, m);

                if (!m.isColliding)
                {
                    continue;
                }

                colliderA->PopulateContactPoints(colliderB.get(), m);

                if (isTriggerInvolved)
                {
                    //notify the event-bus
                    raiseEvent<trigger_event>(&m, m_timeStep);
                    //notify both the trigger and triggerer
                    //TODO:(Developer-The-Great): the triggerer and trigger should probably received this event
                    //TODO:(cont.) through the event bus, we should probably create a filterable system here to
                    //TODO:(cont.) uniquely identify involved objects and then redirect only required messages
                }

                if (isRigidbodyInvolved && !isTriggerInvolved)
                {
                    raiseEvent<collision_event>(&m, m_timeStep);
                    manifoldsToSolve.emplace_back(std::move(m));
                }
            }
        }
    }
}
