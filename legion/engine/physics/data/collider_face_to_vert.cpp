#include <physics/data/collider_face_to_vert.hpp>
#include <physics/physics_statics.hpp>

namespace legion::physics
{
    std::pair<math::vec3, float> legion::physics::ColliderFaceToVert::getFurthestOutsideVert() const
    {
        const math::vec3& faceCentroid = face->centroid;
        const math::vec3& faceNormal = face->normal;

        float largestDist = std::numeric_limits<float>::lowest();
        const math::vec3* mostDistantVert = nullptr;

        for (const math::vec3& vert : outsideVerts)
        {
            float distance = PhysicsStatics::PointDistanceToPlane(faceNormal,faceCentroid,vert);

            if (distance > largestDist)
            {
                largestDist = distance;
                mostDistantVert = &vert;
            }
        }

        return std::make_pair(*mostDistantVert,largestDist);
    }
    void ColliderFaceToVert::populateVectorWithVerts(std::vector<math::vec3>& vertVector)
    {
        auto& vec = outsideVerts;
        auto cap = outsideVerts.capacity();
        auto size = outsideVerts.size();
       
        for (size_t i = 0; i < outsideVerts.size(); i++)
        {
            vertVector.push_back(outsideVerts.at(i));
        }

    }
}

