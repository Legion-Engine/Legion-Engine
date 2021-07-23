
#include <physics/data/convexconvexpenetrationquery.hpp>
#include <physics/physics_statics.hpp>
#include <physics/physics_contact.hpp>
#include <physics/data/contact_vertex.hpp>

namespace legion::physics
{
    ConvexConvexPenetrationQuery::ConvexConvexPenetrationQuery(HalfEdgeFace* pRefFace, HalfEdgeFace* pIncFace,
        math::vec3& pFaceCentroid, math::vec3& pNormal, float pPenetration, bool pIsARef)
        :  PenetrationQuery(pFaceCentroid,pNormal,pPenetration,pIsARef),refFace(pRefFace), incFace(pIncFace)
    {
        debugID = "ConvexConvexPenetrationQuery";
    }

    void ConvexConvexPenetrationQuery::populateContactList(physics_manifold& manifold, math::mat4& refTransform
        , math::mat4 incTransform, PhysicsCollider* refCollider)
    {
        auto incCollider = isARef ? manifold.colliderB : manifold.colliderA;
        float largestDotResult = std::numeric_limits<float>::lowest();

        //------------------------------- find face that is touching refFace -------------------------------------------------//

        for (auto face : incCollider->GetHalfEdgeFaces())
        {
            math::vec3 worldFaceNormal = incTransform * math::vec4(face->normal, 0);
            float currentDotResult = math::dot(-normal, worldFaceNormal);
            if (currentDotResult > largestDotResult)
            {
                largestDotResult = currentDotResult;
                incFace = face;
            }
        }
 
        //------------------------------- get all world vertex positions in incFace -------------------------------------------------//
        std::vector<ContactVertex> outputContactPoints;

        bool facePartiallyBelowPlane = false;

        auto sendToInitialOutput = [&outputContactPoints,&incTransform](HalfEdgeEdge* edge)
        {
            math::vec3 localVertexPosition = edge->edgePosition;
            math::vec3 worldVertex = incTransform * math::vec4(localVertexPosition, 1);

            outputContactPoints.push_back(ContactVertex(worldVertex,edge->label));
        };

        incFace->forEachEdge(sendToInitialOutput);

        //------------------------------- clip vertices with faces that are the neighbors of refFace  ---------------------------------//
        auto clipNeigboringFaceWithOutput = [&refTransform,&outputContactPoints](HalfEdgeEdge* edge)
        {
            HalfEdgeFace* neighborFace = edge->pairingEdge->face;
            math::vec3 planePosition = refTransform * math::vec4(neighborFace->centroid, 1);
            math::vec3 planeNormal = refTransform * math::vec4(neighborFace->normal, 0);

            auto inputContactList = outputContactPoints;
            outputContactPoints.clear();


            PhysicsStatics::SutherlandHodgmanFaceClip(planeNormal, planePosition, inputContactList, outputContactPoints,edge);

        };

        refFace->forEachEdge(clipNeigboringFaceWithOutput);

        for (const auto& incidentContact : outputContactPoints)
        {
            float distanceToCollisionPlane = PhysicsStatics::PointDistanceToPlane(normal, faceCentroid, incidentContact.position);
            
            if (distanceToCollisionPlane < constants::contactOffset)
            {
                math::vec3 referenceContact = incidentContact.position - normal * distanceToCollisionPlane;

                physics_contact contact;
                contact.refCollider = refCollider;
                contact.IncWorldContact = incidentContact.position;
                contact.RefWorldContact = referenceContact;
                contact.label = incidentContact.label;

                refCollider->AttemptFindAndCopyConverganceID(contact);

                manifold.contacts.push_back(contact);
            }
        }
    }
}
