#pragma once
#include <physics/halfedgeface.hpp>
#include <core/core.hpp>

namespace legion::physics
{
    class ColliderFaceToVert
    {
    public:

        std::vector<math::vec3> outsideVerts;
        HalfEdgeFace* face = nullptr;

        ColliderFaceToVert(HalfEdgeFace* pFace) : face(pFace)
        {
            pFace->faceToVert = this;
        }

        ~ColliderFaceToVert()
        {

        }

        ColliderFaceToVert(const ColliderFaceToVert& other)
        {
            outsideVerts = other.outsideVerts;
            face = other.face;
        }

        std::pair<math::vec3,float> getFurthestOutsideVert() const;
        
        void populateVectorWithVerts(std::vector<math::vec3>& vertVector);

        
    };
}
