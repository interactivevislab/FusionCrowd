#include "WayPortal.h"



WayPortal::WayPortal(const NavMeshEdge * edge, unsigned int nodeID, bool p0IsLeft) :
	_edge(edge), _nodeID(nodeID), _p0IsLeft(p0IsLeft) {
}

void WayPortal::setPreferredDirection(const FusionCrowd::Math::Vector2 & pos, float radius, const FusionCrowd::Math::Vector2 & dir,
	Agents::PrefVelocity & pVel) const
{
	_edge->setClearDirections(pos, radius, dir, pVel);

}

FusionCrowd::Math::Vector2 WayPortal::intersectionPoint(const FusionCrowd::Math::Vector2 & point, const FusionCrowd::Math::Vector2 & dir) const
{
	FusionCrowd::Math::Vector2 pDir = _edge->getDirection();
	FusionCrowd::Math::Vector2 p0 = _edge->getP0();
	float denom = det(pDir, dir);

	float num = det(dir, p0 - point);
	float s = num / denom;
	return p0 + s * pDir;
}


WayPortal::~WayPortal()
{
}
