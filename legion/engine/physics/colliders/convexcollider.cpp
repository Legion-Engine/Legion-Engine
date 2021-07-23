#include <physics/colliders/convexcollider.hpp>
#include <physics/physics_statics.hpp>
#include <physics/data/identifier.hpp>
#include <physics/data/convexconvexpenetrationquery.hpp>
#include <physics/data/edgepenetrationquery.hpp>
#include <physics/data/pointer_encapsulator.hpp>
#include <physics/systems/physicssystem.hpp>
#include <rendering/debugrendering.hpp>

namespace legion::physics
{
    void ConvexCollider::CheckCollisionWith(ConvexCollider* convexCollider, physics_manifold& manifold) 
    {
        OPTICK_EVENT();

        // Middle-phase collision detection
        // Do AABB collision to check whether collision is possible
        auto aabbThis = this->GetMinMaxWorldAABB();
        auto aabbOther = convexCollider->GetMinMaxWorldAABB();
        auto& [low0, high0] = aabbThis;
        auto& [low1, high1] = aabbOther;
       
        if (!physics::PhysicsStatics::CollideAABB(low0, high0, low1, high1))
        {

            manifold.isColliding = false;
            return;
        }

        //--------------------- Check for a collision by going through the edges and faces of both polyhedrons  --------------//
        //'this' is colliderB and 'convexCollider' is colliderA
        
        PointerEncapsulator < HalfEdgeFace> ARefFace;

        float ARefSeperation;
        if (PhysicsStatics::FindSeperatingAxisByExtremePointProjection(
            this, convexCollider, manifold.transformB,manifold.transformA,  ARefFace, ARefSeperation) || !ARefFace.ptr)
        {
            manifold.isColliding = false;
            return;
        }

        PointerEncapsulator < HalfEdgeFace> BRefFace;

        float BRefSeperation;
        if (PhysicsStatics::FindSeperatingAxisByExtremePointProjection(convexCollider,
            this, manifold.transformA, manifold.transformB, BRefFace, BRefSeperation) || !BRefFace.ptr)
        {
            manifold.isColliding = false;
            return;
        }

        PointerEncapsulator< HalfEdgeEdge> edgeRef;
        PointerEncapsulator< HalfEdgeEdge> edgeInc;

        math::vec3 edgeNormal;
        float aToBEdgeSeperation;

        if (PhysicsStatics::FindSeperatingAxisByGaussMapEdgeCheck( this, convexCollider, manifold.transformB, manifold.transformA,
            edgeRef, edgeInc, edgeNormal, aToBEdgeSeperation,true ) || !edgeRef.ptr )
        {
            manifold.isColliding = false;
            return;
        }

        //--------------------- A Collision has been found, find the most shallow penetration  ------------------------------------//

        //TODO all penetration querys should supply a constructor that takes in a  ConvexConvexCollisionInfo
        
        math::vec3 worldFaceCentroidA = manifold.transformA * math::vec4(ARefFace.ptr->centroid, 1);
        math::vec3 worldFaceNormalA = manifold.transformA * math::vec4(ARefFace.ptr->normal, 0);
        
        math::vec3 worldFaceCentroidB = manifold.transformB * math::vec4(BRefFace.ptr->centroid, 1);
        math::vec3 worldFaceNormalB = manifold.transformB * math::vec4(BRefFace.ptr->normal, 0);

        math::vec3 worldEdgeAPosition = edgeRef.ptr? manifold.transformB * math::vec4(edgeRef.ptr->edgePosition, 1) : math::vec3();
        math::vec3 worldEdgeNormal = edgeNormal;

        auto abPenetrationQuery =
            std::make_unique< ConvexConvexPenetrationQuery>(ARefFace.ptr
                , BRefFace.ptr, worldFaceCentroidA,worldFaceNormalA, ARefSeperation,true);

        auto baPenetrationQuery =
            std::make_unique < ConvexConvexPenetrationQuery>(BRefFace.ptr, ARefFace.ptr,
                worldFaceCentroidB, worldFaceNormalB, BRefSeperation, false);

        auto abEdgePenetrationQuery = 
            std::make_unique < EdgePenetrationQuery>(edgeRef.ptr, edgeInc.ptr,worldEdgeAPosition,worldEdgeNormal,
                aToBEdgeSeperation, false);

        //-------------------------------------- Choose which PenetrationQuery to use for contact population --------------------------------------------------//

        if (abPenetrationQuery->penetration + physics::constants::faceToFacePenetrationBias >
            baPenetrationQuery->penetration)
        {
            manifold.penetrationInformation = std::move(abPenetrationQuery);
        }
        else
        {
            manifold.penetrationInformation = std::move(baPenetrationQuery);
        }


        if (abEdgePenetrationQuery->penetration >
            manifold.penetrationInformation->penetration + physics::constants::faceToEdgePenetrationBias)
        {
            manifold.penetrationInformation = std::move(abEdgePenetrationQuery);
        }

        manifold.isColliding = true;
    }

    void ConvexCollider::PopulateContactPointsWith(ConvexCollider* convexCollider, physics_manifold& manifold)
    {
        OPTICK_EVENT();
        math::mat4& refTransform = manifold.penetrationInformation->isARef ? manifold.transformA : manifold.transformB;
        math::mat4& incTransform = manifold.penetrationInformation->isARef ? manifold.transformB : manifold.transformA;

        physicsComponent* refPhysicsComp = manifold.penetrationInformation->isARef ? manifold.physicsCompA : manifold.physicsCompB;
        physicsComponent* incPhysicsComp = manifold.penetrationInformation->isARef ? manifold.physicsCompB : manifold.physicsCompA;

        PhysicsCollider* refCollider = manifold.penetrationInformation->isARef ? manifold.colliderA : manifold.colliderB;
        PhysicsCollider* incCollider = manifold.penetrationInformation->isARef ? manifold.colliderB : manifold.colliderA;

        manifold.penetrationInformation->populateContactList(manifold, refTransform, incTransform, refCollider);

        rigidbody* refRB = manifold.penetrationInformation->isARef ? manifold.rigidbodyA : manifold.rigidbodyB;
        rigidbody* incRB = manifold.penetrationInformation->isARef ? manifold.rigidbodyB : manifold.rigidbodyA;

        math::vec3 refWorldCentroid = refTransform * math::vec4(refPhysicsComp->localCenterOfMass,1);
        math::vec3 incWorldCentroid = incTransform * math::vec4(incPhysicsComp->localCenterOfMass,1);

        for ( auto& contact : manifold.contacts)
        {
            contact.incTransform = incTransform;
            contact.refTransform = refTransform;

            contact.rbInc = incRB;
            contact.rbRef = refRB;
           
            contact.collisionNormal = manifold.penetrationInformation->normal;

            contact.refRBCentroid = refWorldCentroid;
            contact.incRBCentroid = incWorldCentroid;

        }
    }

    void ConvexCollider::UpdateTightAABB(const math::mat4& transform)
    {
        minMaxWorldAABB = PhysicsStatics::ConstructAABBFromTransformedVertices
        (vertices, transform);
    }

    void ConvexCollider::UpdateLocalAABB()
    {
        minMaxLocalAABB = PhysicsStatics::ConstructAABBFromVertices(vertices);
    }

    void ConvexCollider::DrawColliderRepresentation(const math::mat4& transform,math::color usedColor, float width, float time,bool ignoreDepth)
    {
        if (!shouldBeDrawn) { return; }
        //math::vec3 colliderCentroid = pos + math::vec3(localTransform * math::vec4(physCollider->GetLocalCentroid(), 0));
        //debug::user_projectDrawLine(colliderCentroid, colliderCentroid + math::vec3(0.0f,0.2f,0.0f), math::colors::cyan, 6.0f,0.0f,true);

        for (auto face : GetHalfEdgeFaces())
        {
            //face->forEachEdge(drawFunc);
            physics::HalfEdgeEdge* initialEdge = face->startEdge;
            physics::HalfEdgeEdge* currentEdge = face->startEdge;

            math::vec3 faceStart = transform * math::vec4(face->centroid, 1);
            math::vec3 faceEnd = faceStart + math::vec3((transform * math::vec4(face->normal, 0))) * 0.5f;

            debug::user_projectDrawLine(faceStart, faceEnd, math::colors::green, 2.0f);

            if (!currentEdge) { return; }

            do
            {
                physics::HalfEdgeEdge* edgeToExecuteOn = currentEdge;
                currentEdge = currentEdge->nextEdge;

                math::vec3 worldStart = transform * math::vec4(edgeToExecuteOn->edgePosition, 1);
                math::vec3 worldEnd = transform * math::vec4(edgeToExecuteOn->nextEdge->edgePosition, 1);

                debug::user_projectDrawLine(worldStart, worldEnd, usedColor, width, time,ignoreDepth);

            } while (initialEdge != currentEdge && currentEdge != nullptr);
        }


    }

  /*  void ConvexCollider::ConstructConvexHullWithMesh(legion::core::mesh_handle meshHandle, math::mat4 DEBUG_transform)
    {
        meshHandle.
        return PhysicsStatics::generateConvexHull()
    }*/

    void ConvexCollider::ConstructConvexHullWithMesh(mesh& mesh, math::vec3 spacingAmount,bool shouldDebug)
    {
        OPTICK_EVENT();
       // log::debug("-------------------------------- ConstructConvexHullWithMesh ----------------------------------");
        // Step 0 - Create inital hull
        /*if (step == 0)
        {*/
        //Make sure our mesh has enough vertices for at least an initial hulls
        if (mesh.vertices.size() < 4)
        {
            log::warn("Hull generation skipped, because mesh had less than 4 verticess");
            return;
        }
        
        //index 0 and 1 - a line
        auto [index0, index1] = convexHullFindOuterIndices(mesh);

        /*if (step == 0)
        {
            return;
        }*/
        //index 2 - together with index0 and index1 creates a plane 
        size_type index2 = convexHullFindIndexClosestToLine(mesh, index0, index1);
        if (index2 == mesh.vertices.size())
        {
            log::error("Initial hull, index 2 cannot be found!");
            return;
        }

        //normal for face constructed from index0, index1 and index2
        math::vec3 normal012 = math::normalize(math::cross((mesh.vertices.at(index1) - mesh.vertices.at(index0)), (mesh.vertices.at(index2) - mesh.vertices.at(index0))));
        if (debug)
        {
            log::debug("triangle verts {} {} {} ", mesh.vertices.at(index0), mesh.vertices.at(index1), mesh.vertices.at(index2));
        }
        vertices.push_back(mesh.vertices.at(index0));
        vertices.push_back(mesh.vertices.at(index1));
        vertices.push_back(mesh.vertices.at(index2));
        HalfEdgeEdge* edge0 = new HalfEdgeEdge(vertices.at(0));
        HalfEdgeEdge* edge1 = new HalfEdgeEdge(vertices.at(1));
        HalfEdgeEdge* edge2 = new HalfEdgeEdge(vertices.at(2));
        edge0->setNextAndPrevEdge(edge2, edge1); //goes from 0 to 1
        edge1->setNextAndPrevEdge(edge0, edge2); //goes from 1 to 2
        edge2->setNextAndPrevEdge(edge1, edge0); //goes from 2 to 0
        HalfEdgeFace* face012 = new HalfEdgeFace(edge0, normal012);
        halfEdgeFaces.push_back(face012);
        faceIndexMap.emplace(halfEdgeFaces[0], 0);

       /* if (step == 1)
        {
            return;
        }*/

        //index 3 - Together with previous indices creates initial hull
        size_type index3 = convexHullFindIndexClosestToTriangle(mesh, index0, index1, index2, normal012, debug);


        if (index3 == mesh.vertices.size())
        {
            log::error("Initial hull, index 3 cannot be found!");
            return;
        }

        // Build the initial hull
#pragma region buildInitialHull
   /*             vertices.push_back(mesh.vertices.at(index0));
                vertices.push_back(mesh.vertices.at(index1));
                vertices.push_back(mesh.vertices.at(index2));*/
        vertices.push_back(mesh.vertices.at(index3));

        // Face 0 - edges: 0, 1, 2 - vertices: 0, 1, 2
        //HalfEdgeEdge* edge0 = new HalfEdgeEdge(vertices.at(0));
        //HalfEdgeEdge* edge1 = new HalfEdgeEdge(vertices.at(1));
        //HalfEdgeEdge* edge2 = new HalfEdgeEdge(vertices.at(2));
        //edge0->setNextAndPrevEdge(edge2, edge1); //goes from 0 to 1
        //edge1->setNextAndPrevEdge(edge0, edge2); //goes from 1 to 2
        //edge2->setNextAndPrevEdge(edge1, edge0); //goes from 2 to 0
        //HalfEdgeFace* face012 = new HalfEdgeFace(edge0, normal012);

        // Face 1 - edges: 3, 4, 5 - vertices: 3, 1, 0
        HalfEdgeEdge* edge3 = new HalfEdgeEdge(vertices.at(3));
        HalfEdgeEdge* edge4 = new HalfEdgeEdge(vertices.at(1));
        HalfEdgeEdge* edge5 = new HalfEdgeEdge(vertices.at(0));
        edge3->setNextAndPrevEdge(edge5, edge4); // goes from 3 to 1
        edge4->setNextAndPrevEdge(edge3, edge5); // goes from 1 to 0
        edge5->setNextAndPrevEdge(edge4, edge3); // goes from 0 to 3
        math::vec3 normal310 = math::normalize(math::cross((mesh.vertices.at(index1) - mesh.vertices.at(index3)), (mesh.vertices.at(index0) - mesh.vertices.at(index3))));
        HalfEdgeFace* face310 = new HalfEdgeFace(edge3, normal310);


        // Face 2 - edges: 6, 7, 8 - vertices 2, 1, 3
        HalfEdgeEdge* edge6 = new HalfEdgeEdge(vertices.at(2));
        HalfEdgeEdge* edge7 = new HalfEdgeEdge(vertices.at(1));
        HalfEdgeEdge* edge8 = new HalfEdgeEdge(vertices.at(3));
        edge6->setNextAndPrevEdge(edge8, edge7); // goes from 2 to 1
        edge7->setNextAndPrevEdge(edge6, edge8); // goes from 1 to 3
        edge8->setNextAndPrevEdge(edge7, edge6); // goes from 3 to 2
        math::vec3 normal213 = math::normalize(math::cross((mesh.vertices.at(index1) - mesh.vertices.at(index2)), (mesh.vertices.at(index3) - mesh.vertices.at(index2))));
        HalfEdgeFace* face213 = new HalfEdgeFace(edge6, normal213);


        // Face 3 - edges: 9, 10, 11 - vertices: 3, 0, 2
        HalfEdgeEdge* edge9 = new HalfEdgeEdge(vertices.at(3));
        HalfEdgeEdge* edge10 = new HalfEdgeEdge(vertices.at(0));
        HalfEdgeEdge* edge11 = new HalfEdgeEdge(vertices.at(2));
        edge9->setNextAndPrevEdge(edge11, edge10); // goes from 3 to 0
        edge10->setNextAndPrevEdge(edge9, edge11); // goes from 0 to 2
        edge11->setNextAndPrevEdge(edge10, edge9); // goes from 2 to 3
        math::vec3 normal302 = math::normalize(math::cross((mesh.vertices.at(index0) - mesh.vertices.at(index3)), (mesh.vertices.at(index2) - mesh.vertices.at(index3))));
        HalfEdgeFace* face302 = new HalfEdgeFace(edge9, normal302);

        // Pair edges
        edge0->setPairingEdge(edge4);
        edge1->setPairingEdge(edge6);
        edge2->setPairingEdge(edge10);
        edge3->setPairingEdge(edge7);
        edge5->setPairingEdge(edge9);
        edge8->setPairingEdge(edge11);

        // Make sure all the created faces are convex with each other
        // Also invert normals when they are concave

        HalfEdgeFace::makeNormalsConvexWithFace(*face012, *face310);
        HalfEdgeFace::makeNormalsConvexWithFace(*face012, *face213);
        HalfEdgeFace::makeNormalsConvexWithFace(*face012, *face302);
        HalfEdgeFace::makeNormalsConvexWithFace(*face310, *face213);
        HalfEdgeFace::makeNormalsConvexWithFace(*face310, *face302);
        HalfEdgeFace::makeNormalsConvexWithFace(*face213, *face302);

        // Add faces to the faces list
        //halfEdgeFaces.push_back(face012);
        halfEdgeFaces.push_back(face310);
        halfEdgeFaces.push_back(face213);
        halfEdgeFaces.push_back(face302);

        // Store face index in faceIndexMap
        //faceIndexMap.emplace(halfEdgeFaces[0], 0);
        faceIndexMap.emplace(halfEdgeFaces[1], 1);
        faceIndexMap.emplace(halfEdgeFaces[2], 2);
        faceIndexMap.emplace(halfEdgeFaces[3], 3);

#pragma endregion buildInitalHull
        // End of building the initial hull

        //  Construct a toBeSorted list, conaining all the vertices that need to be added to the mesh
        for (int i = 0; i < mesh.vertices.size(); ++i)
        {
            bool skipVert = false;
            for (int j = 0; j < vertices.size(); ++j)
                if (mesh.vertices[i] == vertices[j]) skipVert = true;
            if (skipVert) continue;

            toBeSorted.push_back(mesh.vertices.at(i));
        }

        if (shouldDebug)
        {
            DrawColliderRepresentation(math::mat4(1.0f), math::colors::green, 12.0f, FLT_MAX);
        }
        


        ///}
        // end of Step 0 - Create inital hull

        // Step 1 and up, add a vert to the hull
       /* if (step == 2) return;
        looped = 2;
        while (looped < step)*/
        int iter = 0;
        while (true)
        {
            //log::debug(" iter");
            /*if (looped == step - 1)
            {
                log::debug("--------------------------------------------------------------------------------------");
                log::debug("--------------------------------------- STEP {} ---------------------------------------", step);
                log::debug("--------------------------------------------------------------------------------------------");
            }*/

            // Section here is return condition
            if (toBeSorted.size() == 0)
            {
                // We are done here
                // if toBesorted == 0 there are no vertices left in the original mesh that are not added to the hulls

                break;
            }

            faceVertMap.clear();
            faceVertMap = convexHullMatchVerticesToFace(toBeSorted);

            // Check if there are any faces with matched unsorted vertices
            bool shouldEnd = true;
            for (auto& faceVert : faceVertMap)
            {

                if (faceVert.size() != 0)
                {
                    shouldEnd = false;
                    break;
                }
            }

          


            if (shouldEnd)
            {
                //We are done yay!
                // If there aren't any unsorted vertices, we should exit 
                break;

            }

            // Find the largest distance form vert to its face
            // That vert will be added to the hull
            float largestDistance = 0;
            size_type faceIndex = 0;
            size_type vertIndex = 0;
            for (size_type i = 0; i < faceVertMap.size(); ++i)
            {
                for (size_type j = 0; j < faceVertMap.at(i).size(); ++j)
                {
                    // Technically we need the distance toward a face, not necessarily a triangle
                    // Because the vert was macthed to this face, it probably does not matter
                    float dist = math::pointToTriangle(faceVertMap.at(i).at(j), halfEdgeFaces.at(i)->startEdge->edgePosition, halfEdgeFaces.at(i)->startEdge->nextEdge->edgePosition, halfEdgeFaces.at(i)->startEdge->prevEdge->edgePosition, halfEdgeFaces.at(i)->normal);
                    if (dist > largestDistance)
                    {
                        largestDistance = dist;
                        faceIndex = i;
                        vertIndex = j;
                    }
                }
            }


            // list of horizon edges
            std::deque<HalfEdgeEdge*> edges;
            // The selected vert needs to be merged into the hull
            // Find the horizon edges for the current vertex
            //convexHullConstructHorizon(faceVertMap.at(faceIndex).at(vertIndex), *halfEdgeFaces.at(faceIndex),edges);
            ConstructHorizonByEdgeJumping(faceVertMap.at(faceIndex).at(vertIndex), edges, halfEdgeFaces,spacingAmount,
                shouldDebug && iter == 7);

            //if (looped == step - 1)
            //{
            //    auto& face = *halfEdgeFaces.at(faceIndex);

            //    auto prevStart = face.startEdge->prevEdge->edgePosition;
            //    auto start = face.centroid + spacingAmount;
            //    auto end = face.startEdge->nextEdge->edgePosition;
      
            //    debug::user_projectDrawLine(start,
            //        start + face.normal * 0.2f, math::colors::blue, 20.0f, FLT_MAX);

            //}


            if (looped == step - 1)
            {
                //log::debug("------ Printing Normals -------------");
                float max = edges.size()-1;
                //log::debug("  edges size {} ", edges.size());
                for (size_t i = 0; i < edges.size(); i++)
                {
                    
                    float interpolant = (float)i / max;
                    math::vec3 edgePosition = edges.at(i)->edgePosition;
                    math::vec3 nextEdgePosiion = edges.at((i + 1) % edges.size())->edgePosition;
                    math::vec3 vecColor = math::vec3(1, 0, 0) * interpolant;

                    //log::debug("Normal {}",edges.at(i)->face->normal);

                    debug::user_projectDrawLine(edgePosition + spacingAmount,
                        nextEdgePosiion + spacingAmount, math::color(vecColor.x, vecColor.y, vecColor.z,1), 25.0f, FLT_MAX,true);

                }



            }

            HalfEdgeEdge* firstEdge = nullptr;
            HalfEdgeEdge* pairingEdge = nullptr;

            std::unordered_set<HalfEdgeFace*> facesToBeDeleted;
            std::vector<HalfEdgeFace*> createdFaces;

            // Create new faces for each edge
            for (int i = 0; i < edges.size(); ++i)
            {
                // Do not yet delete the face which will be overlapped by the new face
                // We will store it, and delete it later
                facesToBeDeleted.emplace(edges.at(i)->face);

                // Create the edges for the new face of the hull
                HalfEdgeEdge* edge0 = new HalfEdgeEdge(edges.at(i)->edgePosition);
                HalfEdgeEdge* edge1 = new HalfEdgeEdge(edges.at(i)->nextEdge->edgePosition); // Tail of edge
                HalfEdgeEdge* edge2 = new HalfEdgeEdge(faceVertMap.at(faceIndex).at(vertIndex)); // Vertex positon


                // Setup next and previous edges of the edges we just created
                edge0->setNextAndPrevEdge(edge2, edge1);
                edge1->setNextAndPrevEdge(edge0, edge2);
                edge2->setNextAndPrevEdge(edge1, edge0);

                // We know the pairing edge of edge0, because edge0 is the same as the horizonEdge
                // Therefore the pairing edge of the horizon edge will be the pairing edge of egde0
                edge0->setPairingEdge(edges.at(i)->pairingEdge);


                // If this is the first iteration, we need to remember this edge
                // This edge2 will be used later to as pairing edge
                // Otherwise edge2 can set its pairing to the previous edge1 (saved as pairingEdge)
                if (i == 0)
                {
                    firstEdge = edge2;
                }
                else
                {
                    edge2->setPairingEdge(pairingEdge);
                }

                // On the last iteration, we can set the pairing Edge of edge1 to firstEdge
                // Which is the edge we stored at the beginning (edge2 of first iteration)
                if (i == edges.size() - 1)
                {
                    edge1->setPairingEdge(firstEdge);
                }

                // Save pairingEdge
                pairingEdge = edge1;

                // Calculate normal
                math::vec3 normal = math::normalize(math::cross(edge1->edgePosition - edge0->edgePosition, edge2->edgePosition - edge0->edgePosition));

                //Create Face
                HalfEdgeFace* face = new HalfEdgeFace(edge0, normal);
                face->setFaceForAllEdges();

                //Add Face to Faces
                createdFaces.push_back(face);
            }

            // Add the vertex we added to the hull, to our vertices list
            vertices.push_back(faceVertMap.at(faceIndex).at(vertIndex));

            // Store our added vert
            math::vec3 addedVert = faceVertMap.at(faceIndex).at(vertIndex);

            // Clear the toBeSorted list, so we can fill it again with vertices that need to be sorted
            // We can not just remove the added vertex since there may be vertices that share the same positions
            toBeSorted.clear();
            // We add all the vertices to the toBeSorted list that are in the faceVertMap that are not equal to addedVert
            for (size_type i = 0; i < faceVertMap.size(); ++i)
            {
                for (size_type j = 0; j < faceVertMap.at(i).size(); ++j)
                {
                    // Make sure we do not add the vert to the list that we just added to the hull
                    if (faceVertMap.at(i).at(j) != addedVert)
                        toBeSorted.push_back(faceVertMap.at(i).at(j));
                }
            }

            // Now the old edges edges and faces that have been replaced with new faces will be deleted
            // Edges will be deleted by the destructor of HalfEdgeFace
            for (auto& face : facesToBeDeleted)
            {
                // Remove face from faces map
                halfEdgeFaces.erase(std::remove(halfEdgeFaces.begin(), halfEdgeFaces.end(), face), halfEdgeFaces.end());
                // Remove face from our face to face index map
                faceIndexMap.erase(faceIndexMap.find(face));
                face->startEdge = nullptr;
                //delete face;
            }
            //log::debug("----------------------------------------------------");
            // Some of the created faces may be coplanar to other faces, these faces will be merged
            convexHullMergeFaces(createdFaces);

            // Make sure all the normals are correct
            for (int i = 0; i < createdFaces.size(); ++i)
            {
                // Add new faces to faceIndexMap
                faceIndexMap.emplace(createdFaces.at(i), halfEdgeFaces.size());
                // Add new faces to halfEdgeFaces vector
                halfEdgeFaces.push_back(createdFaces.at(i));

                // We only need to check convexity with the very next face
                // Therefore we cross over the startEdge to startEdge.pairingEdge and take that face
                HalfEdgeFace* pairing = createdFaces.at(i)->startEdge->pairingEdge->face;
                createdFaces.at(i)->makeNormalsConvexWithNeighbors(*pairing);

                //HalfEdgeFace::makeNormalsConvexWithFace(*createdFaces.at(i), *pairing);
            }
            // We increase looped counter for debug tools
            ++looped;
            iter++;
        }

        //log::debug("---------------------------------------------------------------------");

        //std::vector<HalfEdgeFace*> removed;
        //for (size_t i = 0; i < halfEdgeFaces.size(); i++)
        //{
        //   /* HalfEdgeFace* pairing = halfEdgeFaces.at(i)->startEdge->pairingEdge->face;
        //    HalfEdgeFace::makeNormalsConvexWithFace(*halfEdgeFaces.at(i), *pairing);*/
        //    halfEdgeFaces.at(i)->mergeCoplanarNeighbors(removed);
        //}

        //for (size_t i = 0; i < removed.size(); i++)
        //{
        //    std::vector<HalfEdgeFace*>::iterator iter;

        //    iter = std::remove(halfEdgeFaces.begin(), halfEdgeFaces.end(), removed.at(i));
        //}
        ////convexHullMergeFaces(halfEdgeFaces,true);
        AssertEdgeValidity();
        //log::debug("-> Finish ConstructConvexHullWithMesh ----------------------------------");
    }

    void ConvexCollider::PopulateVertexListWithHalfEdges()
    {
        auto& verticesVec = vertices;

        int reserveSize = halfEdgeFaces.size() * 3;

        verticesVec.reserve(reserveSize);


        auto collectVertices = [&verticesVec](HalfEdgeEdge* edge)
        {
            edge->calculateRobustEdgeDirection();
            verticesVec.push_back(edge->edgePosition -= PhysicsStatics::PointDistanceToPlane(edge->face->normal, edge->face->centroid, edge->edgePosition));
        };

        for (auto face : halfEdgeFaces)
        {
            face->forEachEdge(collectVertices);
        }


    }

    void ConvexCollider::convexHullConstructHorizon(math::vec3 vert, HalfEdgeFace& face, std::deque<HalfEdgeEdge*>& edges, HalfEdgeEdge* originEdge,
        std::shared_ptr<std::unordered_set<HalfEdgeFace*>> visited)
    {
        if (!visited)
        {
            visited = std::make_shared<std::unordered_set<HalfEdgeFace*>>();
            float distanceToPlane =
                math::pointToPlane(vert, face.startEdge->edgePosition, face.normal);

            //log::debug("Start Face Dist {} ", distanceToPlane);
        }

        // Make sure we do not call recursively on faces we came from
        if (visited->find(&face) != visited->end()) return;
        // Mark face as "visited"
        visited->emplace(&face);
        HalfEdgeEdge* start = face.startEdge;
        HalfEdgeEdge* edge = start;
        

        // Loop through all the edges of this face
        do
        {

            // Makr sure we do not keep jumping between 2 faces (over 2 edges)
            if (originEdge == edge)
            {
                edge = edge->nextEdge;
                continue;
            }

            if (!edge->pairingEdge)
            {
                edge = edge->nextEdge;
                log::debug("-> Pairing Edge was null");
                continue;
            }
            // Cross the edge -> check next face

            //if (debugDraw)
            //{

            //    math::color debugColor = math::colors::white;
            //    edge->DEBUG_drawInsetEdge(spacing, debugColor * depth, FLT_MAX, 20.0f);
            //}

            HalfEdgeFace* crossedFace = edge->pairingEdge->face;
            if (!crossedFace->startEdge) return;

            float distanceToPlane =
                math::pointToPlane(vert, crossedFace->startEdge->edgePosition, crossedFace->normal);

            log::debug(" crossedFace normal {}", crossedFace->normal);
            log::debug("distanceToPlane {} ", distanceToPlane);
            // If the vertex is above the face (The face can see the vertex), the face is part of the horizon
            if (distanceToPlane > math::epsilon<float>())
            {
                log::debug("\t Checking Next Face");
                // Plane is part of horizon -> find horizon edges recursively
                convexHullConstructHorizon(vert, *crossedFace, edges, edge->pairingEdge, visited);

            }
            else
            {
                log::debug("\t Adding Edge");
                // Plane is not part of horizon
                // Therefore the crossed edge was a horizon edge
                edges.push_back(edge);
            }

            edge = edge->nextEdge;
        } while (edge != start);
    }

    void ConvexCollider::ConstructHorizonByEdgeJumping(math::vec3 vert, std::deque<HalfEdgeEdge*>& edges
        , std::vector<HalfEdgeFace*>& faces, math::vec3 spacing, bool shouldDebug )
    {
        //log::debug("-> ConstructHorizonByEdgeJumping");
        //from list of faces, get first horizon edge
        HalfEdgeEdge* firstHorizonEdge = nullptr;
        
        for (auto face : faces)
        {
            auto findHorizonLambda = [vert,&firstHorizonEdge](HalfEdgeEdge* currentEdge)
            {
                if (!firstHorizonEdge)
                {
                    if (currentEdge->isEdgeHorizonFromVertex(vert))
                    {
                        firstHorizonEdge = currentEdge;
                    }
                }
            };

            face->forEachEdge(findHorizonLambda);
        }


        if (firstHorizonEdge)
        {
            if (shouldDebug)
            {
                firstHorizonEdge->DEBUG_drawInsetEdge(spacing, math::colors::darkgrey, FLT_MAX, 12.0f);
            }
        }
        else
        {
            //log::error("Horizon Edge NOT FOUND!");
            return;
        }

        HalfEdgeEdge* iterEdge = firstHorizonEdge;

        int i = 0;
        int j = 0;

        do
        {
            edges.push_back(iterEdge);

            iterEdge = iterEdge->nextEdge;

            while (!iterEdge->isEdgeHorizonFromVertex(vert))
            {
                iterEdge = iterEdge->pairingEdge;
                iterEdge = iterEdge->nextEdge;

                j++;

                if (j > 999)
                {
                    edges.clear();
                    log::error("Quickhull: Stuck in while loop at horizon jumping");
                    return;
                }
            }
            j = 0;

            i++;
            assert(i < 999);
        } while (iterEdge != firstHorizonEdge);
    }

}

