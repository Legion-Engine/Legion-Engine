#include <physics/halfedgeedge.hpp>
#include <physics/halfedgeface.hpp>
#include <physics/data/collider_face_to_vert.hpp>
#include <physics/physics_statics.hpp>

namespace legion::physics
{
    HalfEdgeEdge::HalfEdgeEdge(math::vec3 newEdgePositionPtr) : edgePosition{ newEdgePositionPtr }
    {
        static int idCount = 0;
        identifier = idCount;

        idCount++;
    }
    void HalfEdgeEdge::setNextAndPrevEdge(HalfEdgeEdge* newPrevEdge, HalfEdgeEdge* newNextEdge)
    {
        nextEdge = newNextEdge;
        prevEdge = newPrevEdge;
    }

    void HalfEdgeEdge::setNext(HalfEdgeEdge* newNextEdge)
    {
        nextEdge = newNextEdge;
        newNextEdge->prevEdge = this;

    }

    void HalfEdgeEdge::setPrev(HalfEdgeEdge* newPrevEdge)
    {
        prevEdge = newPrevEdge;
        newPrevEdge->nextEdge = this;
    }

    void HalfEdgeEdge::setPairingEdge(HalfEdgeEdge* edge)
    {
        pairingEdge = edge;
        edge->pairingEdge = this;
    }

    math::vec3 HalfEdgeEdge::getLocalNormal() const
    {
        return face->normal;
    }

    void HalfEdgeEdge::calculateRobustEdgeDirection() 
    {
        math::vec3 firstNormal = face->normal;
        math::vec3 secondNormal = pairingEdge->face->normal;

        robustEdgeDirection = math::cross(firstNormal, secondNormal);
    }

    bool HalfEdgeEdge::isVertexVisible(const math::vec3& vert,float epsilon)
    {
        float distanceToPlane =
            math::pointToPlane(vert, face->centroid, face->normal);

        return distanceToPlane > epsilon;
    }

    bool HalfEdgeEdge::isEdgeHorizonFromVertex(const math::vec3& vert, float epsilon)
    {
        return isVertexVisible(vert,epsilon) && !pairingEdge->isVertexVisible(vert,epsilon);
    }

    void HalfEdgeEdge::DEBUG_drawEdge(const math::mat4& transform, const math::color& debugColor, float time, float width)
    {
        math::vec3 worldStart = transform * math::vec4(edgePosition, 1);
        math::vec3 worldEnd = transform * math::vec4(nextEdge->edgePosition, 1);

        debug::user_projectDrawLine(worldStart, worldEnd, debugColor, width, time, true);
    }

    void HalfEdgeEdge::DEBUG_drawInsetEdge(const math::vec3 spacing, const math::color& debugColor, float time, float width)
    {
        math::vec3 worldCentroid = face->centroid + spacing;

        math::vec3 worldStart = edgePosition + spacing;
        math::vec3 startDifference = (worldCentroid - worldStart) * 0.1f;

        math::vec3 worldEnd = nextEdge->edgePosition + spacing;
        math::vec3 endDifference = (worldCentroid - worldEnd) * 0.1f;

        debug::user_projectDrawLine(worldStart + startDifference, worldEnd + endDifference, debugColor, width, time, true);
    }

    void HalfEdgeEdge::DEBUG_directionDrawEdge(const math::mat4& transform, const math::color& debugColor, float time, float width)
    {
        math::vec3 worldStart = transform * math::vec4(edgePosition, 1);
        math::vec3 worldEnd = transform * math::vec4(nextEdge->edgePosition, 1);

        math::vec3 worldCentroid = transform * math::vec4(face->centroid, 1);

        math::vec3 startDifference = (worldCentroid - worldStart) * 0.1f;
        math::vec3 endDifference = (worldCentroid - worldEnd) * 0.1f;

        debug::user_projectDrawLine(worldStart + startDifference, worldEnd + endDifference, debugColor, width, time, true);

        math::vec3 pointStart = worldStart + startDifference;
        math::vec3 diff = worldEnd + endDifference - (worldStart + startDifference);

        debug::user_projectDrawLine(pointStart + diff * 0.75f, worldCentroid, math::colors::red, width, time, true);
    }

    void HalfEdgeEdge::suicidalMergeWithPairing(std::vector<math::vec3>& unmergedVertices, math::vec3& normal, float scalingEpsilon)
    {
        //[1] identify connecting edges of this face and merge face and connect them together
        //[2] Handle possible issue where merging this face and merge faces causes the new face to have 2 edges with the same neighbors
        //[3] re-Initialize to account for new edges
        //[4] delete pairing and 'this' edge. These edges are no longer part of this face


        auto releaseFaceToVert = [&unmergedVertices](HalfEdgeFace* face)
        {
            ColliderFaceToVert* otherFaceToVert = face->faceToVert;
            otherFaceToVert->populateVectorWithVerts(unmergedVertices);
            otherFaceToVert->face = nullptr;
        };

        //[1] identify connecting edges of this face and merge face and connect them together
        //------------------------------------------------------------------//
        //                                                                  //
        // prevFromCurrentConnection    prevFromCurrent                     //
        //       ___|_______________   ___|_______________                  //
        //                          | |                                     //
        //                         -| |                                     //
        //                   'this' | | pairingEdge                         //
        //                          | |-                                    //
        //                          | |                                     //
        //       _____________|_____| |____________|______                  //
        // nextFromCurrentConnection    nextFromCurrent                     //
        //                                                                  //
        //       [face of 'this']           [merge face]                    //
        //                                                                  //
        //------------------------------------------------------------------//

        HalfEdgeFace* mergeFace = pairingEdge->face;
        releaseFaceToVert(mergeFace);
        
        HalfEdgeEdge* prevFromCurrent =  pairingEdge->prevEdge;
        HalfEdgeEdge* prevFromCurrentConnection = prevFromCurrent->nextEdge->pairingEdge->nextEdge;
        prevFromCurrent->setNext(prevFromCurrentConnection); 

        HalfEdgeEdge* nextFromCurrent =  pairingEdge->nextEdge;
        HalfEdgeEdge* nextFromCurrentConnection = nextFromCurrent->prevEdge->pairingEdge->prevEdge;

        nextFromCurrent->setPrev(nextFromCurrentConnection); 

        face->startEdge = prevFromCurrent;
        face->normal = normal;

        mergeFace->startEdge = nullptr;
        delete mergeFace;

        //[2] Handle possible issue where merging this face and merge faces causes the new face to have 2 edges that neighbor the same face

        //----------------------------------------------------------------------//
        //                        [invariantMergeFace]                          //
        //   |                                                  |               //
        //   |newPrevFromFCC                      newNextFromPFC| -             //
        //   |                                                  |               //
        //  -|                                                  |               //
        //   |_________________________|_   _________________|__|               //
        //    ___|________________________   ___|_______________                //
        //   |  prevFromCurrentConnection | | prevFromCurrent   |               //
        //   |                           -| |                   |-              //
        //   |                     'this' | | pairingEdge       | prevFromPFC   //
        //   | nextFromFCC                | |-                  |               //
        //  -|                            | |                   |               //
        //   |______________________|_____| |____________|______|               //
        //       nextFromCurrentConnection    nextFromCurrent                   //
        //                                                                      //
        //             [face of 'this']           [merge face]                  //
        //                                                                      //
        //----------------------------------------------------------------------//

        auto handleDoubleAdjacentMergeResult = [this,releaseFaceToVert](HalfEdgeEdge* prevFromCurrent, HalfEdgeEdge* prevFromCurrentConnection)
        {
            HalfEdgeFace* invariantMergeFace = prevFromCurrent->pairingEdge->face;
            releaseFaceToVert(invariantMergeFace);

            HalfEdgeEdge* prevFromPFC = prevFromCurrent->prevEdge;
            HalfEdgeEdge* newNextFromPFC = prevFromCurrent->pairingEdge->nextEdge;
            prevFromPFC->setNext(newNextFromPFC); 

            HalfEdgeEdge* nextFromFCC = prevFromCurrentConnection->nextEdge;
            HalfEdgeEdge* newPrevFromFCC = prevFromCurrentConnection->pairingEdge->prevEdge;
            nextFromFCC->setPrev(newPrevFromFCC); 

            delete prevFromCurrent->pairingEdge;
            delete prevFromCurrent;

            delete prevFromCurrentConnection->pairingEdge;
            delete prevFromCurrentConnection;

            face->startEdge = prevFromPFC;

            std::vector<math::vec3> vertices;
            auto collectVertices = [&vertices](HalfEdgeEdge* currentEdge) { vertices.push_back(currentEdge->edgePosition); };
            face->forEachEdge(collectVertices);

            float tempDis;
            PhysicsStatics::calculateNewellPlane(vertices, face->normal, tempDis);

            invariantMergeFace->startEdge = nullptr;
            delete invariantMergeFace;
        };

        HalfEdgeFace* currentFace = face;
        auto setEdgeFace = [&currentFace](HalfEdgeEdge* edge) {edge->face = currentFace; };

        if (prevFromCurrent->pairingEdge->face == prevFromCurrentConnection->pairingEdge->face)
        {
            face->forEachEdge(setEdgeFace);

            math::vec3 norm;
            if (PhysicsStatics::isNewellFacesCoplanar(face, prevFromCurrent->pairingEdge->face, prevFromCurrent, scalingEpsilon, norm,2))
            {
                handleDoubleAdjacentMergeResult(prevFromCurrent, prevFromCurrentConnection);
            }
        }

        if (nextFromCurrent->pairingEdge->face == nextFromCurrentConnection->pairingEdge->face)
        {
            face->forEachEdge(setEdgeFace);

            math::vec3 norm;
            if (PhysicsStatics::isNewellFacesCoplanar(face, nextFromCurrent->pairingEdge->face, nextFromCurrentConnection, scalingEpsilon, norm,2))
            {
                handleDoubleAdjacentMergeResult(nextFromCurrentConnection, nextFromCurrent);
            }
        }

        //[3] re-Initialize to account for new edges
        face->initializeFace();

        //[4] delete pairing and 'this' edge. These edges are no longer part of this face
        delete pairingEdge;
        delete this;

    }
}

