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

        /** @brief Given the positions in 'outsideVerts', gets the vertex furthest from the HalfEdgeFace 'face'
         */
        std::pair<math::vec3,float> getFurthestOutsideVert() const;
       
        /** @brief Given the vertices in 'vertVector', adds them to 'outsideVerts'
         */
        void populateVectorWithVerts(std::vector<math::vec3>& vertVector);

        
    };
}
