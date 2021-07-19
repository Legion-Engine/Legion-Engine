#pragma once
#include <core/core.hpp>
#include <physics/data/edge_label.hpp>

#include <rendering/debugrendering.hpp>

namespace legion::physics
{
    struct HalfEdgeFace;

	struct HalfEdgeEdge
	{
		HalfEdgeEdge* pairingEdge = nullptr;
		HalfEdgeEdge* nextEdge = nullptr;
		HalfEdgeEdge* prevEdge = nullptr;

		HalfEdgeFace* face = nullptr;

        EdgeLabel label;
        
		math::vec3 edgePosition;
        std::string id;

        HalfEdgeEdge() = default;
        int identifier;

        HalfEdgeEdge(math::vec3 newEdgePositionPtr);

		/**@brief sets 'nextEdge' and 'prevEdge' with the given HalfEdgeEdges
		*/
        void setNextAndPrevEdge(HalfEdgeEdge* newPrevEdge, HalfEdgeEdge* newNextEdge);

        void setNext(HalfEdgeEdge* newNextEdge);
        void setPrev(HalfEdgeEdge* newPrevEdge);
		

        /**@brief Sets the pairingEdge to the passed in edge, and sets the edge.pairingEdge to this
         * if this edge or the passed edge already have pairing edge, the pairing edge of the pairing edge will be set to nullptr
         */
        void setPairingEdge(HalfEdgeEdge* edge);
       

        math::vec3 getLocalNormal() const;
		

		/**@brief Gets the direction of the edge by getting the
		* vector starting from the current edge's position to the next edge
		*/
        math::vec3 getLocalEdgeDirection()  const;
		

        bool isVertexVisible(const math::vec3& vert, float epsilon = math::sqrt(math::epsilon<float>()));
        

        bool isEdgeHorizonFromVertex(const math::vec3& vert,float epsilon = math::sqrt(math::epsilon<float>()));
       

        void DEBUG_drawEdge(const math::mat4& transform, const math::color& debugColor, float time = 20.0f, float width = 5.0f);
        

        void DEBUG_drawInsetEdge(const math::vec3 spacing, const math::color& debugColor, float time = 20.0f, float width = 5.0f);

        void DEBUG_directionDrawEdge(const math::mat4& transform, const math::color& debugColor, float time, float width);

        void suicidalMergeWithPairing(std::vector<math::vec3>&unmergedVertices,math::vec3& normal,float scalingEpsilon, math::mat4 transform = math::mat4(1.0),bool shouldDebug = false);


	};
}

