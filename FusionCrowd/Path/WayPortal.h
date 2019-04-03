#pragma once

#include "../NavComponents/NavMesh/NavMeshEdge.h"
#include "../Math/vector.h"
#include "../Config.h"
#include "PrefVelocity.h"

class PortalRoute;
class PortalPath;

class FUSION_CROWD_API WayPortal
{
public:
	WayPortal(const NavMeshEdge * edge, unsigned int nodeID, bool p0IsLeft);

	inline FusionCrowd::Math::Vector2 getLeft() const {
		return _p0IsLeft ? _edge->getP0() : _edge->getP1();
	}

	inline FusionCrowd::Math::Vector2 getLeft(float offset) const {
		return _p0IsLeft ? _edge->getP0(offset) : _edge->getP1(offset);
	}

	inline FusionCrowd::Math::Vector2 getRight() const {
		return _p0IsLeft ? _edge->getP1() : _edge->getP0();
	}

	inline FusionCrowd::Math::Vector2 getRight(float offset) const {
		return _p0IsLeft ? _edge->getP1(offset) : _edge->getP0(offset);
	}

	FusionCrowd::Math::Vector2 intersectionPoint(const FusionCrowd::Math::Vector2 & point,
		const FusionCrowd::Math::Vector2 & dir) const;

	void setPreferredDirection(const FusionCrowd::Math::Vector2 & pos, float radius,
		const FusionCrowd::Math::Vector2 & dir, Agents::PrefVelocity & pVel) const;

	friend class PortalRoute;
	friend class PortalPath;

	~WayPortal();

protected:
	const NavMeshEdge *	_edge;
	unsigned int _nodeID;
	bool	_p0IsLeft;
};

