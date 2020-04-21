#include "WayPortal.h"
#include "Math/Util.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	WayPortal::WayPortal(const NavMeshEdge* edge, unsigned int nodeID, bool p0IsLeft) :
		_edge(edge), _nodeID(nodeID), _p0IsLeft(p0IsLeft)
	{
	}

	void WayPortal::setPreferredDirection(const Vector2& pos, float radius, const Vector2& dir,
	                                      Agents::PrefVelocity& pVel) const
	{
		_edge->setClearDirections(pos, radius, dir, pVel);
	}

	Vector2 WayPortal::intersectionPoint(const Vector2& point, const Vector2& dir) const
	{
		Vector2 pDir = _edge->getDirection();
		Vector2 p0 = _edge->getP0();
		float denom = FusionCrowd::Math::det(pDir, dir);

		float num = FusionCrowd::Math::det(dir, p0 - point);
		float s = num / denom;
		return p0 + s * pDir;
	}


	WayPortal::~WayPortal()
	{
	}
}
