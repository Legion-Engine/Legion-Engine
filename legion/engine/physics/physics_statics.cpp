#include <core/core.hpp>
#include <physics/physics_statics.hpp>
#include <rendering/debugrendering.hpp>

namespace legion::physics
{
    bool PhysicsStatics::FindSeperatingAxisByExtremePointProjection(ConvexCollider* convexA
        , ConvexCollider* convexB, const math::mat4& transformA, const math::mat4& transformB,
        PointerEncapsulator<HalfEdgeFace>& refFace,
        float& maximumSeperation, bool shouldDebug )
    {
        float currentMaximumSeperation = std::numeric_limits<float>::lowest();

        for (auto face : convexB->GetHalfEdgeFaces())
        {

            math::vec3 seperatingAxis = math::normalize(transformB * math::vec4((face->normal), 0));

            math::vec3 transformedPositionB = transformB * math::vec4(face->centroid, 1);

            //get extreme point of other face in normal direction
            math::vec3 worldSupportPoint; 
            GetSupportPointNoTransform(transformedPositionB, -seperatingAxis,
                convexA, transformA, worldSupportPoint);

            float seperation = math::dot(worldSupportPoint - transformedPositionB, seperatingAxis);

            if (seperation > currentMaximumSeperation)
            {
                currentMaximumSeperation = seperation;
                refFace.ptr = face;
            }

            if (seperation > 0)
            {
                //we have found a seperating axis, we can exit early
                maximumSeperation = currentMaximumSeperation;
                return true;
            }
        }

        //no seperating axis was found
        maximumSeperation = currentMaximumSeperation;

        return false;
    }

    float PhysicsStatics::GetSupportPoint(const std::vector<math::vec3>& vertices, const math::vec3& direction,math::vec3& outVec)
    {
        float currentMaximumSupportPoint = std::numeric_limits<float>::lowest();

        for (const auto& vert : vertices)
        {
            float dotResult = math::dot(direction, vert);

            if (dotResult > currentMaximumSupportPoint)
            {
                currentMaximumSupportPoint = dotResult;
                outVec = vert;
            }
        }

        return currentMaximumSupportPoint;
    }

    void PhysicsStatics::GetSupportPointNoTransform(math::vec3 planePosition, math::vec3 direction, ConvexCollider* collider, const math::mat4& colliderTransform
        , math::vec3& worldSupportPoint)
    {
        float largestDistanceInDirection = std::numeric_limits<float>::lowest();
        planePosition = math::inverse(colliderTransform) * math::vec4(planePosition, 1);
        direction = math::inverse(colliderTransform) * math::vec4(direction, 0);

        const auto& vertices = collider->GetVertices();

        for (int vertexIndex = 0 ; vertexIndex < vertices.size(); ++vertexIndex)
        {
            math::vec3 transformedVert = math::vec4(vertices.at(vertexIndex), 1);

            float dotResult = math::dot(transformedVert - planePosition, direction);

            if (dotResult > largestDistanceInDirection)
            {
                largestDistanceInDirection = dotResult;
                worldSupportPoint = transformedVert;
            }
        }

        worldSupportPoint = colliderTransform * math::vec4(worldSupportPoint, 1);
    }

    bool PhysicsStatics::FindSeperatingAxisByGaussMapEdgeCheck(ConvexCollider* convexA, ConvexCollider* convexB, const math::mat4& transformA,
        const math::mat4& transformB, PointerEncapsulator<HalfEdgeEdge>& refEdge, PointerEncapsulator<HalfEdgeEdge>& incEdge,
        math::vec3& seperatingAxisFound, float& maximumSeperation, bool shouldDebug)
    {
        float currentMaximumSeperation = std::numeric_limits<float>::lowest();

        math::vec3 centroidDir = transformA * math::vec4(convexA->GetLocalCentroid(), 0);
        math::vec3 positionA = math::vec3(transformA[3]) + centroidDir;

        int facei = 0;
        int facej = 0;

        for (const auto faceA : convexA->GetHalfEdgeFaces())
        {
            //----------------- Get all edges of faceA ------------//
            std::vector<HalfEdgeEdge*> convexAHalfEdges;

            auto lambda = [&convexAHalfEdges](HalfEdgeEdge* edge)
            {
                convexAHalfEdges.push_back(edge);
            };

            faceA->forEachEdge(lambda);
            facej = 0;


            for (const auto faceB : convexB->GetHalfEdgeFaces())
            {
                //----------------- Get all edges of faceB ------------//
                std::vector<HalfEdgeEdge*> convexBHalfEdges;

                auto lambda = [&convexBHalfEdges](HalfEdgeEdge* edge)
                {
                    convexBHalfEdges.push_back(edge);
                };

                faceB->forEachEdge(lambda);

                bool isCorrectFaces = facei == 0 && facej == 13;

                int edgeAIter = 0;

                for (HalfEdgeEdge* edgeA : convexAHalfEdges)
                {
                    bool drawA = isCorrectFaces && edgeAIter == 1;

                    if (drawA)
                    {
                        //edgeA->DEBUG_drawEdge(transformA, math::colors::magenta, 5.0f);
                    }

                    int edgeBIter = 0;

                    for (HalfEdgeEdge* edgeB : convexBHalfEdges)
                    {
                        bool drawB = isCorrectFaces && edgeBIter == 0;

                        if (drawB)
                        {
                            //edgeB->DEBUG_drawEdge(transformB, math::colors::cyan, 5.0f);
                        }

                        edgeBIter++;

                        //if the given edges creates a minkowski face
                        if (attemptBuildMinkowskiFace(edgeA, edgeB, transformA, transformB))
                        {
                            //get world edge direction
                            math::vec3 edgeADirection = transformA * math::vec4(edgeA->getRobustEdgeDirection(), 0);
                            math::vec3 edgeBDirection = transformB * math::vec4(edgeB->getRobustEdgeDirection(), 0);

                            edgeADirection = math::normalize( edgeADirection );
                            edgeBDirection = math::normalize( edgeBDirection );

                            static float angleThres = math::cos(math::deg2rad(3.0f));
                            float angle = math::abs( math::dot(edgeADirection, edgeBDirection) );

                            if ( angle > angleThres )
                            {
                                continue;
                            }

                            math::vec3 seperatingAxis = math::normalize(math::cross(edgeADirection, edgeBDirection));

                            //get world edge position
                            math::vec3 edgeAtransformedPosition = transformA * math::vec4(edgeA->edgePosition, 1);
                            math::vec3 edgeBtransformedPosition = transformB * math::vec4(edgeB->edgePosition, 1);

                            //check if its pointing in the right direction 
                            if (math::dot(seperatingAxis, edgeAtransformedPosition - positionA) < 0)
                            {
                                seperatingAxis = -seperatingAxis;
                            }

                            //check if given edges create a seperating axis
                            float distance = math::dot(seperatingAxis, edgeBtransformedPosition - edgeAtransformedPosition);
                            //log::debug("distance {} , currentMinimumSeperation {}", distance, currentMinimumSeperation);
                            if (distance > currentMaximumSeperation)
                            {
                                refEdge.ptr = edgeA;
                                incEdge.ptr = edgeB;

                                seperatingAxisFound = seperatingAxis;
                                currentMaximumSeperation = distance;
                                maximumSeperation = currentMaximumSeperation;
                            }

                            if (distance > 0)
                            {
                                return true;
                            }

                        }
                        
                    }

                    edgeAIter++;
                }
                facej++;
            }
            facei++;
        }

        return false;
    }

    bool PhysicsStatics::DetectConvexSphereCollision(ConvexCollider* convexA, const math::mat4& transformA, math::vec3 sphereWorldPosition, float sphereRadius,
        float& maximumSeperation)
    {
        //-----------------  check if the seperating axis is the line generated between the centroid of the hull and sphereWorldPosition ------------------//

        math::vec3 worldHullCentroid = transformA * math::vec4(convexA->GetLocalCentroid(), 1);
        math::vec3 centroidSeperatingAxis = math::normalize(worldHullCentroid - sphereWorldPosition);

        math::vec3 seperatingPlanePosition = sphereWorldPosition + centroidSeperatingAxis * sphereRadius;

        math::vec3 worldSupportPoint;
        GetSupportPoint(seperatingPlanePosition, -centroidSeperatingAxis, convexA, transformA, worldSupportPoint);

        float seperation = math::dot(worldSupportPoint - seperatingPlanePosition, centroidSeperatingAxis);

        if (seperation > 0.0f)
        {
            maximumSeperation = seperation;
            return false;
        }

        maximumSeperation = std::numeric_limits<float>::lowest();

        //--------------------------------- check if the seperating axis one of the faces of the convex hull ----------------------------------------------//

        for (auto faceA : convexA->GetHalfEdgeFaces())
        {
            math::vec3 worldFaceCentroid = transformA * math::vec4(faceA->centroid, 1);
            math::vec3 worldFaceNormal = math::normalize(transformA * math::vec4(faceA->normal, 0));

            float seperation = PointDistanceToPlane(worldFaceNormal, worldFaceCentroid, seperatingPlanePosition );

            if (seperation > maximumSeperation)
            {
                maximumSeperation = seperation;
            }

            if (seperation > sphereRadius)
            {
                return false;
            }

        }


        return true;
    }

    std::pair< math::vec3, math::vec3> PhysicsStatics::ConstructAABBFromPhysicsComponentWithTransform
    (ecs::component_handle<physicsComponent> physicsComponentToUse,const math::mat4& transform)
    {
        math::vec3 min, max;

        //auto physicsComponent = physicsComponentToUse.read();

        ////get up
        //math::vec3 invTransUp = math::normalize( math::inverse(transform) * math::vec4(math::vec3(0, 1, 0), 0) );
        //max.y = GetPhysicsComponentSupportPointAtDirection(invTransUp, physicsComponent);
        //
        ////get down
        //math::vec3 invTransDown = math::normalize(math::inverse(transform) * math::vec4(math::vec3(0, -1, 0), 0));
        //min.y = GetPhysicsComponentSupportPointAtDirection(invTransUp, physicsComponent);

        ////get right


        ////get left


        ////get forward


        ////get backward


        return std::make_pair(min,max);
    }

    float PhysicsStatics::GetPhysicsComponentSupportPointAtDirection(math::vec3 direction, physicsComponent& physicsComponentToUse)
    {
        float currentMaximumSupportPoint = std::numeric_limits<float>::lowest();

        //std::vector<math::vec3> vertices;
        ////for each vertex list of each collider
        //for (auto collider : *physicsComponentToUse.colliders)
        //{
        //    auto [first,second] = collider->GetminMaxLocalAABB();
        //    vertices.push_back(first);
        //    vertices.push_back(second);
        //}

        //for (const auto& vert : vertices)
        //{
        //    float dotResult = math::dot(direction, vert);

        //    if (dotResult > currentMaximumSupportPoint)
        //    {
        //        currentMaximumSupportPoint = dotResult;
        //    }
        //}

        return currentMaximumSupportPoint;
    }

    std::pair<math::vec3, math::vec3> PhysicsStatics::ConstructAABBFromVertices(const std::vector<math::vec3>& vertices)
    {
        math::vec3 min, max;

        ////up
        //max.y = GetSupportPoint(vertices, math::vec3(0, 1, 0));
        ////down
        //min.y = GetSupportPoint(vertices, math::vec3(0, -1, 0));

        ////right
        //max.x = GetSupportPoint(vertices, math::vec3(1, 0, 0));
        ////left
        //min.x = GetSupportPoint(vertices, math::vec3(-1, 0, 0));
   

        ////forward
        //max.z = GetSupportPoint(vertices, math::vec3(0, 0, 1));
        ////backward
        //min.z = GetSupportPoint(vertices, math::vec3(0, 0, -1));

        return std::make_pair(min,max);
    }

    std::pair<math::vec3, math::vec3> PhysicsStatics::ConstructAABBFromTransformedVertices(const std::vector<math::vec3>& vertices, const math::mat4& transform)
    {
        math::vec3 min, max;
        math::vec3 worldPos = transform[3];

        math::vec3 outVec;
        //up
        math::vec3 invTransUp = math::normalize(math::inverse(transform) * math::vec4(0, 1, 0, 0));
        GetSupportPoint(vertices, invTransUp, outVec);
        max.y = (transform * math::vec4( outVec,1)).y;

        //down
        math::vec3 invTransDown = math::normalize(math::inverse(transform) * math::vec4(0, -1, 0, 0));
        GetSupportPoint(vertices, invTransDown, outVec);
        min.y = (transform * math::vec4(outVec, 1)).y;

        //right
        math::vec3 invTransRight = math::normalize(math::inverse(transform) * math::vec4(1, 0, 0, 0));
        GetSupportPoint(vertices, invTransRight, outVec);
        max.x = (transform * math::vec4(outVec, 1)).x;

        //left
        math::vec3 invTransLeft = math::normalize(math::inverse(transform) * math::vec4(-1, 0, 0, 0));
        GetSupportPoint(vertices, invTransLeft, outVec);
        min.x = (transform * math::vec4(outVec, 1)).x;

        //forward
        math::vec3 invTransForward = math::normalize(math::inverse(transform) * math::vec4(0, 0, 1, 0));
        GetSupportPoint(vertices, invTransForward, outVec);
        max.z = (transform * math::vec4(outVec, 1)).z;

        //backward
        math::vec3 invTransBackward = math::normalize(math::inverse(transform) * math::vec4(0, 0, -1, 0));
        GetSupportPoint(vertices, invTransBackward, outVec);
        min.z = (transform * math::vec4(outVec, 1)).z;

      

        return std::make_pair(min, max);
    }

    std::pair<math::vec3, math::vec3> PhysicsStatics::CombineAABB(const std::pair<math::vec3, math::vec3>& first, const std::pair<math::vec3, math::vec3>& second)
    {
        auto& firstLow = first.first;
        auto& firstHigh = first.second;
        auto& secondLow = second.first;
        auto& secondHigh = second.second;
        math::vec3 lowBounds = secondLow;
        math::vec3 highBounds = secondHigh;
        if (firstLow.x < secondLow.x)   lowBounds.x    = firstLow.x;
        if (firstLow.y < secondLow.y)   lowBounds.y    = firstLow.y;
        if (firstLow.z < secondLow.z)   lowBounds.z    = firstLow.z;
        if (firstHigh.x > secondHigh.x) highBounds.x   = firstHigh.x;
        if (firstHigh.y > secondHigh.y) highBounds.y   = firstHigh.y;
        if (firstHigh.z > secondHigh.z) highBounds.z   = firstHigh.z;

        return std::make_pair(lowBounds, highBounds);
    }

    float PhysicsStatics::FindClosestPointToLineInterpolant(const math::vec3& startPoint, const math::vec3& lineDirection, const math::vec3& pointPosition)
    {
        return ( math::dot(lineDirection,pointPosition) - math::dot(lineDirection,startPoint) ) / math::dot(lineDirection,lineDirection);
    }

    math::vec3 PhysicsStatics::FindClosestPointToLineSegment(const math::vec3& start, const math::vec3& end, const math::vec3& pointPosition)
    {
        float interpolant = FindClosestPointToLineInterpolant(start, end - start, pointPosition);
        interpolant = math::clamp(interpolant, 0.0f, 1.0f);

        return start + (end - start) * interpolant;
    }

    std::shared_ptr<ConvexCollider> PhysicsStatics::generateConvexHull(const std::vector<math::vec3>& vertices)
    {
        //[1] Calculate scaled epsilon
        const static float initialEpsilon = math::sqrt( math::epsilon<float>() );

        math::vec3 maxInDimension(std::numeric_limits<float>::lowest());

        for (const auto& vert : vertices)
        {
            if (math::abs(vert.x) > maxInDimension.x)
            {
                maxInDimension.x = math::abs(vert.x);
            }

            if (math::abs(vert.y) > maxInDimension.y)
            {
                maxInDimension.y = math::abs(vert.y);
            }

            if (math::abs(vert.z) > maxInDimension.z)
            {
                maxInDimension.z = math::abs(vert.z);
            }
        }
        
        //epsilon must take into account span of vertices
        float dimensionSum = 3 * ( maxInDimension.x + maxInDimension.y + maxInDimension.z );
        const float scaledEpsilon = dimensionSum * initialEpsilon;
        const float visibilityEpsilon = dimensionSum * math::pow(math::epsilon<float>(), 1.0f / 2.5f);

        std::vector<HalfEdgeFace*> faces;
        faces.reserve(4);

        //[3] Measure extents of mesh
        std::array<math::vec3, 6> supportVertices;

        //(1.1) Get support points in -x and x
        GetSupportPoint(vertices, math::vec3(1, 0, 0), supportVertices.at(0));
        GetSupportPoint(vertices, math::vec3(-1, 0, 0), supportVertices.at(1));

        //(1.2) Get support points in -y and y
        GetSupportPoint(vertices, math::vec3(0, 1, 0), supportVertices.at(2));
        GetSupportPoint(vertices, math::vec3(0, -1, 0), supportVertices.at(3));

        //(1.3) Get support points in -z and z
        GetSupportPoint(vertices, math::vec3(0, 0, 1), supportVertices.at(4));
        GetSupportPoint(vertices, math::vec3(0, 0, -1), supportVertices.at(5));

        //[2] Build Initial Hull
        if (!buildInitialHull(vertices, supportVertices, faces,math::mat4(1.0f)))
        {
            return nullptr;
        }
        
        //[3] populate list with faces of initial hull
        std::list<ColliderFaceToVert> facesWithOutsideVerts;
        for (HalfEdgeFace* face : faces)
        {
            facesWithOutsideVerts.emplace_back(face);
        }

        partitionVerticesToList(vertices, facesWithOutsideVerts);

        //[4] loop through faces until there are no faces with unmerged vertices
        if (!facesWithOutsideVerts.empty())
        {
            PointerEncapsulator< ColliderFaceToVert> currentFaceToVert;
            currentFaceToVert.ptr = &facesWithOutsideVerts.front();
   
            while (foundFaceWithOutsideVert(facesWithOutsideVerts, currentFaceToVert))
            {
                //find furhtest vertex of last face
                auto [furthestVert, distanceFromFace] = currentFaceToVert.ptr->getFurthestOutsideVert();

                //check if we should merge this vertex
                if (distanceFromFace > visibilityEpsilon)
                {
                    mergeVertexToHull(furthestVert, facesWithOutsideVerts,scaledEpsilon);
                }
                else
                {
                    //this face has no mergable vertices
                    currentFaceToVert.ptr->outsideVerts.clear();
                }
            }

        }

        auto convexCollider = std::make_shared<ConvexCollider>();
        auto& halfEdgesVector = convexCollider->GetHalfEdgeFaces();

        for (auto& faceToVert : facesWithOutsideVerts)
        {
            halfEdgesVector.push_back(faceToVert.face);
        }
        
        //populate list of vertices in collider list
        convexCollider->PopulateVertexListWithHalfEdges();

        return convexCollider;
    }

    void PhysicsStatics::calculateNewellPlane(const std::vector<math::vec3>& v, math::vec3& outPlaneNormal, float& distToCentroid)
    {
        math::vec3 centroid{0,0,0};
        outPlaneNormal = math::vec3();

        for (int i = v.size() - 1, j = 0; j < v.size(); i = j, j++)
        {
            outPlaneNormal.x += (v[i].y - v[j].y) * (v[i].z + v[j].z); // projection on yz
            outPlaneNormal.y += (v[i].z - v[j].z) * (v[i].x + v[j].x); // projection on xz
            outPlaneNormal.z += (v[i].x - v[j].x) * (v[i].y + v[j].y); // projection on xy
            centroid += v[j];
        }
           
        // Normalize normal and fill in the plane equation fields
        outPlaneNormal = math::normalize(outPlaneNormal);
        distToCentroid = math::dot(centroid, outPlaneNormal) / v.size(); // “centroid / n” is the true centroid point
    }

    bool PhysicsStatics::attemptBuildMinkowskiFace(HalfEdgeEdge* edgeA, HalfEdgeEdge* edgeB, const math::mat4& transformA, const math::mat4& transformB)
    {
        const math::vec3 transformedA1 = transformA *
            math::vec4(edgeA->getLocalNormal(), 0);

        const math::vec3 transformedA2 = transformA *
            math::vec4(edgeA->pairingEdge->getLocalNormal(), 0);

        const math::vec3 transformedEdgeDirectionA =  math::normalize(transformA * math::vec4(edgeA->getRobustEdgeDirection(), 0));

        const math::vec3 transformedB1 = transformB *
            math::vec4(edgeB->getLocalNormal(), 0);

        const math::vec3 transformedB2 = transformB *
            math::vec4(edgeB->pairingEdge->getLocalNormal(), 0);

        const math::vec3 transformedEdgeDirectionB = math::normalize(transformB * math::vec4(edgeB->getRobustEdgeDirection(), 0));

        return isMinkowskiFace(transformedA1, transformedA2, -transformedB1, -transformedB2
            , (transformedEdgeDirectionA), (transformedEdgeDirectionB));
    }

    bool PhysicsStatics::isMinkowskiFace(const math::vec3& transformedA1, const math::vec3& transformedA2, const math::vec3& transformedB1,
        const math::vec3& transformedB2, const math::vec3& planeANormal, const math::vec3& planeBNormal)
    {
        //------------------------ Check if normals created by arcA seperate normals of B --------------------------------------//
        //CBA
        float planeADotB1 = math::dot(planeANormal, transformedB1);
        //DBA
        float planeADotB2 = math::dot(planeANormal, transformedB2);

        float dotMultiplyResultA =
            planeADotB1 * planeADotB2;

        //log::debug("dotMultiplyResultA {}", dotMultiplyResultA);

        if (dotMultiplyResultA > 0.0f )
        {
            return false;
        }

        //------------------------ Check if normals created by arcB seperate normals of A --------------------------------------//

        //ADC
        float planeBDotA1 = math::dot(planeBNormal, transformedA1);
        //BDC
        float planeBDotA2 = math::dot(planeBNormal, transformedA2);

        float  dotMultiplyResultB = planeBDotA1 * planeBDotA2;

        //log::debug("dotMultiplyResultB {}", dotMultiplyResultB);

        if (dotMultiplyResultB > 0.0f )
        {
            return false;
        }

        //------------------------ Check if arcA and arcB are in the same hemisphere --------------------------------------------//

        math::vec3 abNormal = math::cross(transformedA2, transformedB2);

        float planeABDotA1 = math::dot(abNormal, transformedA1);
        float planeABDotB1 = math::dot(abNormal, transformedB1);

        float dotMultiplyResultAB = planeABDotA1 * planeABDotB1;

        if (dotMultiplyResultAB < 0.0f || math::epsilonEqual(dotMultiplyResultAB,0.0f,math::epsilon<float>()))
        {
            return false;
        }

        return true;
    }

    bool PhysicsStatics::buildInitialHull(const std::vector<math::vec3>& vertices,
        std::array<math::vec3,6>& supportVertices, std::vector<HalfEdgeFace*>& faces, math::mat4 DEBUG_transform )
    {
        //Summary:
        //[1] Find the 2 most distant vertices in 'support Vertices'
        //[2] Find the vertex most distant from the line created by the 2 most distance vertices
        //[3] Create first collider face using that line and the vertex most distant to it
        //[4] Find the most distant vertex from the plane where the first collider face lies
        //[5] Create a set of faces connecting the first collider face to the most distant vertex,
        //this create a tetrahedron shaped collider

        //[1] Find the 2 most distant vertices in 'vertices'

        float mostDistant = std::numeric_limits<float>::lowest();

        math::vec3& firstDistant = supportVertices.at(0);
        math::vec3& secondDistant = supportVertices.at(1);

        //(1.1) Iterate through support vertices to find the combination of vertices that represent the 2 most distant points
        for (int i = 0; i < supportVertices.size(); i++)
        {
            math::vec3& first = supportVertices.at(i);

            for (int j = i+1; j < supportVertices.size()-1; j++)
            {
                math::vec3& second = supportVertices.at(j);

                float currentDistance2 = math::distance2(first, second);

                if (currentDistance2 > mostDistant)
                {
                    mostDistant = currentDistance2;
                    firstDistant = first;
                    secondDistant = second;
                }
            }
        }

        //DEBUG draw most distant
      /*  {
            math::vec3 transformFirst = DEBUG_transform * math::vec4(firstDistant, 1);
            math::vec3 transformSecond = DEBUG_transform * math::vec4(secondDistant, 1);

            debug::drawLine(transformFirst, transformSecond,math::colors::red,5.0f,FLT_MAX,true);
        }*/

        //[2] Find the vertex most distant from the line created by the 2 most distance vertices

        math::vec3 firstToSecond = math::normalize(secondDistant - firstDistant);

        const math::vec3* thirdDistant = nullptr;
        mostDistant = std::numeric_limits<float>::lowest();

        //(2.1) Iterate through 'vertices' to find the vertex most distant from line 
        for (auto& vertex : vertices)
        {
            //(2.1.1) Check if vertex and firstDistant create a line segment parralel to the line segment created by 'firstToSecond'
            float dotResult = math::dot(math::normalize(vertex - firstDistant), firstToSecond);

            if (math::close_enough(dotResult, 1.0f))
            {
                continue;
            }

            //(2.1.2) Find closest point between vertex and line segment
            math::vec3 closestPoint = PhysicsStatics::FindClosestPointToLineSegment(firstDistant, secondDistant, vertex);

            float currentDistance = math::distance2(closestPoint, vertex);

            //(2.1.3) Store vertex if it is further than the current known most distant
            if (currentDistance > mostDistant)
            {
                mostDistant = currentDistance;
                thirdDistant = &vertex;
            }
        }

        if (!thirdDistant)
        {
            return false;
        }

        //DEBUG draw most distant
      /*  {
            math::vec3 transformFirst = DEBUG_transform * math::vec4(*thirdDistant, 1);

            debug::drawLine(transformFirst, transformFirst + math::vec3(0,0.1f,0), math::colors::blue, 5.0f, FLT_MAX, true);
        }*/

        //[3] Create first collider face using that line and the vertex most distant to it

        //(3.1) Initialize Half Edges
        HalfEdgeEdge* firstEdge = new HalfEdgeEdge(firstDistant);
        HalfEdgeEdge* secondEdge = new HalfEdgeEdge(secondDistant);
        HalfEdgeEdge* thirdEdge = new HalfEdgeEdge(*thirdDistant);

        //(3.2) Connect them to each other
        firstEdge->setNextAndPrevEdge(thirdEdge, secondEdge);
        secondEdge->setNextAndPrevEdge(firstEdge, thirdEdge);
        thirdEdge->setNextAndPrevEdge(secondEdge, firstEdge);

        //(3.3) Initialize Half Edge Faces
        HalfEdgeFace* initialFace = new HalfEdgeFace(firstEdge,
            math::normalize(math::cross(secondDistant - firstDistant, *thirdDistant - secondDistant)));

        //(3.4) Add to collider
        faces.push_back(initialFace);

        //[4] Find the most distant vertex from the plane where the first collider face lies

        //(4.1) Iterate through vertices to the a point to plane check
        mostDistant = std::numeric_limits<float>::lowest();
        const math::vec3* firstEyePoint = nullptr;

        math::vec3 planePosition = initialFace->centroid;
        math::vec3 planeNormal = initialFace->normal;

        for (auto& vertex : vertices)
        {
            float currentDistance =
                math::abs(PhysicsStatics::PointDistanceToPlane(planeNormal, planePosition, vertex));

            if (currentDistance > mostDistant)
            {
                mostDistant = currentDistance;
                firstEyePoint = &vertex;
            }
        }

        if (math::close_enough(mostDistant,0.0f))
        {
            return false;
        }

        math::color drawC = math::colors::blue;
       

        //[5] invert face if distant vertex is in front of face

        float eyePointDistance =
            PhysicsStatics::PointDistanceToPlane(planeNormal, planePosition, *firstEyePoint);
        bool needInverse = eyePointDistance > 0.0f;
        
        if (needInverse)
        {
            initialFace->inverse();
            drawC = math::colors::red;
        }

        //DEBUG draw most distant
        /*{
            math::vec3 transformFirst = DEBUG_transform * math::vec4(*firstEyePoint, 1);

            debug::drawLine(transformFirst, transformFirst + math::vec3(0, 0.1f, 0), drawC, 5.0f, FLT_MAX, true);

        }*/

        //[5] Create a set of faces connecting the first collider face to the most distant vertex

        //(5.1) Reverse collect the edges of the initialFace

        std::vector<HalfEdgeEdge*> reverseHalfEdgeList;

        auto collectEdges = [&reverseHalfEdgeList](HalfEdgeEdge* current)
        {
            reverseHalfEdgeList.push_back(current);
        };

        initialFace->forEachEdgeReverse(collectEdges);

        //(5.2) For each edge create a new face that connects the initial face with the eyePoint
        math::vec3 eyePoint = *firstEyePoint;

        createHalfEdgeFaceFromEyePoint(eyePoint, reverseHalfEdgeList,faces);

        return true;
    }

    void PhysicsStatics::createHalfEdgeFaceFromEyePoint(const math::vec3 eyePoint, const std::vector<HalfEdgeEdge*>& reversedEdges, std::vector<HalfEdgeFace*>& createdFaces)
    {
        HalfEdgeEdge* pairingToConnectTo = nullptr;
        HalfEdgeEdge* initialPairing = nullptr;

        for (auto edge : reversedEdges)
        {
            //initialize pairing its position is on next
            HalfEdgeEdge* pairing = new HalfEdgeEdge(edge->nextEdge->edgePosition);

            //initialize next pairing its position is on current
            HalfEdgeEdge* nextPairing = new HalfEdgeEdge(edge->edgePosition);

            //initialize prev pairing its position is on the eye point
            HalfEdgeEdge* prevPairing = new HalfEdgeEdge(eyePoint);

            //connect to each other
            pairing->setNextAndPrevEdge(prevPairing, nextPairing);
            nextPairing->setNextAndPrevEdge(pairing, prevPairing);
            prevPairing->setNextAndPrevEdge(nextPairing, pairing);

            //
            pairing->setPairingEdge(edge);

            //initialize new face
            math::vec3 faceNormal = math::normalize(math::cross(nextPairing->edgePosition - pairing->edgePosition,
                prevPairing->edgePosition - pairing->edgePosition));

            HalfEdgeFace* face = new HalfEdgeFace(pairing, faceNormal);

            //push new face into list 
            createdFaces.push_back(face);

            //connect to 
            if (pairingToConnectTo)
            {
                prevPairing->setPairingEdge(pairingToConnectTo);
            }
            else
            {
                initialPairing = prevPairing;
            }

            pairingToConnectTo = nextPairing;
        }

        initialPairing->setPairingEdge(pairingToConnectTo);
    }

    bool PhysicsStatics::foundFaceWithOutsideVert(std::list<ColliderFaceToVert>& facesWithOutsideVerts, PointerEncapsulator< ColliderFaceToVert>& outChosenFace)
    {
        int i = 0;
        for (auto& faceWithOutsideVert : facesWithOutsideVerts)
        {
            if (!faceWithOutsideVert.outsideVerts.empty())
            {
                outChosenFace.ptr = &faceWithOutsideVert;
                return true;
            }
            i++;
        }

        return false;
    }

    void PhysicsStatics::partitionVerticesToList(const std::vector<math::vec3> vertices,
        std::list<ColliderFaceToVert>& outFacesWithOutsideVerts)
    {
        //for each vertex in vertices
        for (const math::vec3& vertex : vertices)
        {
            ColliderFaceToVert* bestFaceToVert = nullptr;
            float bestMatchDistance = std::numeric_limits<float>::lowest();


            for (ColliderFaceToVert& faceToVert : outFacesWithOutsideVerts)
            {
                auto face = faceToVert.face;
                float currentDistance = PointDistanceToPlane(face->normal, face->centroid, vertex);

                if (currentDistance > bestMatchDistance)
                {
                    bestMatchDistance = currentDistance;
                    bestFaceToVert = &faceToVert;

                }
            }

            if (bestFaceToVert && bestMatchDistance > 0.0f)
            {
                bestFaceToVert->outsideVerts.push_back(vertex);
            }
        }
    }

    void PhysicsStatics::findHorizonEdgesFromFaces(const math::vec3& eyePoint, std::vector<HalfEdgeFace*>& faces,
        std::vector<HalfEdgeEdge*>& outHorizonEdges, float scalingEpsilon)
    {
        //[1] Find first horizon edge
        HalfEdgeEdge* initialHorizon = nullptr;
        
        auto findFirstHorizon = [&eyePoint,&initialHorizon,scalingEpsilon](HalfEdgeEdge*edge)
        {
            if (initialHorizon) { return; }
           
            if (edge->isEdgeHorizonFromVertex(eyePoint, scalingEpsilon))
            {
                initialHorizon = edge;
            }
        };

        for (auto face : faces)
        {
            if (initialHorizon) { break; }

            face->forEachEdge(findFirstHorizon);
            
        }

        assert(initialHorizon);

        //[2] Loop through the collider to collect the other horizon edges
       
        HalfEdgeEdge* currentEdge = initialHorizon;

        do
        {
            currentEdge = currentEdge->nextEdge;

            if (!currentEdge->isEdgeHorizonFromVertex(eyePoint, scalingEpsilon))
            {
                auto stuckEdge = currentEdge;
                do
                {
                    auto pairing = currentEdge->pairingEdge;
                    currentEdge = currentEdge->pairingEdge->nextEdge;
                }
                while (!currentEdge->isEdgeHorizonFromVertex(eyePoint, scalingEpsilon));
            }

            outHorizonEdges.push_back(currentEdge);

        } while (currentEdge != initialHorizon);

    }

    void PhysicsStatics::mergeVertexToHull(const math::vec3& eyePoint,std::list<ColliderFaceToVert>& facesWithOutsideVerts,
        float scalingEpsilon)
    {
        std::vector<math::vec3> unmergedVertices;
        std::vector<HalfEdgeFace*> facesToBeRemoved;
          
        //identify faces that can see vertex and remove them from list
        for (auto listIter = facesWithOutsideVerts.begin(); listIter != facesWithOutsideVerts.end();)
        {
            HalfEdgeFace* face = listIter->face;

            const math::vec3& planeCentroid = face->centroid;
            const math::vec3& planeNormal = face->normal;

            float distanceToPlane = PointDistanceToPlane(planeNormal, planeCentroid, eyePoint);

            if (distanceToPlane > scalingEpsilon)
            {
                //face can see vertex, we must remove it from list
                facesToBeRemoved.push_back(face);
                listIter->populateVectorWithVerts(unmergedVertices);
                listIter = facesWithOutsideVerts.erase(listIter);
            }
            else
            {
                listIter++;
            }
        }
        
        //identify horizon edges and put them into list
        std::vector<HalfEdgeEdge*> horizonEdges;
        findHorizonEdgesFromFaces(eyePoint, facesToBeRemoved, horizonEdges,scalingEpsilon);

        //reverse iterate the list to find their pairings, add them to new list
        {
            std::vector<HalfEdgeEdge*> tempEdges = std::move(horizonEdges);
            horizonEdges.clear();

            //what we actually want is its pairings
            for (auto horizonEdge : tempEdges)
            {
                horizonEdges.push_back(horizonEdge->pairingEdge);
            }
        }

        std::vector<HalfEdgeFace*> newFaces;
        newFaces.reserve(horizonEdges.size());
        createHalfEdgeFaceFromEyePoint(eyePoint, horizonEdges, newFaces);

        //merge all new edges until no coplanarity/concavity can be detected

        auto oldEndIter = facesWithOutsideVerts.end();
        oldEndIter--;

        for (HalfEdgeFace* face : newFaces)
        {
            facesWithOutsideVerts.emplace_back(face);
        }

        for (auto listIter = oldEndIter; listIter != facesWithOutsideVerts.end(); listIter++)
        {
            bool mergedInIteration = false;
            if (!listIter->face) {  continue; }

            do
            {
                mergedInIteration = false;

                std::vector<HalfEdgeEdge*> faceEdges;
                faceEdges.reserve(3);

                auto edgeCollect = [&faceEdges](HalfEdgeEdge* edge){ faceEdges.push_back(edge); };

                listIter->face->forEachEdge(edgeCollect);

                for (auto edge : faceEdges)
                {
                    HalfEdgeEdge* pairingEdge = edge->pairingEdge;
                    HalfEdgeFace* currentFace = edge->face;

                    math::vec3 newNormal;

                    if (isNewellFacesCoplanar(currentFace, pairingEdge->face, edge,
                        scalingEpsilon, newNormal,1) || isFacesConcave(edge->face, pairingEdge->face))
                    {
                        edge->suicidalMergeWithPairing(unmergedVertices,newNormal,scalingEpsilon);

                        mergedInIteration = true;
                        break;
                    }
                    else
                    {
                        HalfEdgeEdge* pairing = edge->pairingEdge;
                        HalfEdgeEdge* nextPairing = edge->nextEdge->pairingEdge;

                        if (pairing->face == nextPairing->face)
                        {
                            HalfEdgeEdge* oldNextEdge = edge->nextEdge;
                            edge->setNext(oldNextEdge->nextEdge);

                            HalfEdgeEdge* oldPairingNextEdge = nextPairing->nextEdge;
                            nextPairing->setNext(oldPairingNextEdge->nextEdge);

                            edge->setPairingEdge(nextPairing);

                            edge->face->startEdge = edge;
                            edge->pairingEdge->face->startEdge = nextPairing;

                            delete oldNextEdge;
                            delete oldPairingNextEdge;

                            mergedInIteration = true;
                            break;
                        }
                    }
                }
            } while (mergedInIteration);
        }

        //duplicate faces are generated while face merging, remove them here
        std::set<HalfEdgeFace*> uniqueFaces;
        for (auto listIter = facesWithOutsideVerts.begin(); listIter != facesWithOutsideVerts.end();)
        {
            if (!listIter->face ) { listIter = facesWithOutsideVerts.erase(listIter); continue; } 
            auto iter = uniqueFaces.find(listIter->face);

            if (iter != uniqueFaces.end())
            {
                //this face is a duplicate
                listIter->populateVectorWithVerts(unmergedVertices);
                listIter = facesWithOutsideVerts.erase(listIter);
            }
            else
            {
                //we have not seen this face before
                uniqueFaces.insert(listIter->face);
                listIter++;
            }
        }

        partitionVerticesToList(unmergedVertices, facesWithOutsideVerts);

        for (auto face : facesToBeRemoved)
        {
            delete face;
        }

    }

    bool PhysicsStatics::isFacesConcave(HalfEdgeFace* first, HalfEdgeFace* second)
    {
        float dotResult = math::dot( first->normal, second->centroid - first->centroid );
        return dotResult > 0.0;
    }

    bool PhysicsStatics::isNewellFacesCoplanar(HalfEdgeFace* first, HalfEdgeFace* second,
        HalfEdgeEdge* connectingEdge, float scalingEpsilon,  math::vec3& outNormal,int skipCount)
    {
        //each face will at least have 3 vertices
        std::vector<math::vec3> NewellPolygon;
        NewellPolygon.reserve(6);

        auto firstOriginal = first->startEdge;
        auto secondOriginal = second->startEdge;

        assert(first == connectingEdge->face);

        first->startEdge = connectingEdge; 
        second->startEdge = connectingEdge->pairingEdge;

        for (size_t i = 0; i < skipCount; i++)
        {
            first->startEdge = first->startEdge->nextEdge;
            second->startEdge = second->startEdge->nextEdge;
        }

        auto collectVerticesOfFace = [&NewellPolygon](HalfEdgeEdge* edge)
        {
            NewellPolygon.emplace_back(edge->edgePosition );
        };

        first->forEachEdge(collectVerticesOfFace);
        NewellPolygon.pop_back();

        second->forEachEdge(collectVerticesOfFace);
        NewellPolygon.pop_back();


         float distToCentroid;
        calculateNewellPlane(NewellPolygon, outNormal, distToCentroid);

        float epsilonMultiplier = (first->calculateFaceExtents() + second->calculateFaceExtents()) * 0.5f;

        for (int i = 0; i < NewellPolygon.size(); i++) {
            float dist = math::dot(outNormal, NewellPolygon[i]) - distToCentroid;

            if (math::abs(dist) > scalingEpsilon * epsilonMultiplier)
            {
                return false;
            }
        }

        return true;
    }

};

