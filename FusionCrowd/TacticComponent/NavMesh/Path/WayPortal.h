#pragma once

#include "TacticComponent/PrefVelocity.h"
#include "Navigation/NavMesh/NavMeshEdge.h"
#include "Export/Config.h"
#include "Math/Util.h"

namespace FusionCrowd
{
	class PortalRoute;
	class PortalPath;

	class WayPortal
	{
	public:
		WayPortal(const NavMeshEdge* edge, unsigned int nodeID, bool p0IsLeft);

		inline DirectX::SimpleMath::Vector2 getLeft() const
		{
			return _p0IsLeft ? _edge->getP0() : _edge->getP1();
		}

		inline DirectX::SimpleMath::Vector2 getLeft(float offset) const
		{
			return _p0IsLeft ? _edge->getP0(offset) : _edge->getP1(offset);
		}

		inline DirectX::SimpleMath::Vector2 getRight() const
		{
			return _p0IsLeft ? _edge->getP1() : _edge->getP0();
		}

		inline DirectX::SimpleMath::Vector2 getRight(float offset) const
		{
			return _p0IsLeft ? _edge->getP1(offset) : _edge->getP0(offset);
		}

		DirectX::SimpleMath::Vector2 intersectionPoint(const DirectX::SimpleMath::Vector2& point,
		                                               const DirectX::SimpleMath::Vector2& dir) const;

		void setPreferredDirection(const DirectX::SimpleMath::Vector2& pos, float radius,
		                           const DirectX::SimpleMath::Vector2& dir, Agents::PrefVelocity& pVel) const;

		friend class PortalRoute;
		friend class PortalPath;

		~WayPortal();

	protected:
		const NavMeshEdge* _edge;
		unsigned int _nodeID;
		bool _p0IsLeft;
	};
}
