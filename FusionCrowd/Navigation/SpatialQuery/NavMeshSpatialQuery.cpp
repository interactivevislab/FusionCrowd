#include "NavMeshSpatialQuery.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	NavMeshSpatialQuery::NavMeshSpatialQuery(std::shared_ptr<NavMeshLocalizer> nml)
	{
		_localizer = nml;
		ProcessObstacles();
	}

	bool NavMeshSpatialQuery::QueryVisibility(const Vector2& q1, const Vector2& q2, float radius) const
	{
		return true;
	}

	void NavMeshSpatialQuery::ProcessObstacles()
	{
		// Compute obstacle convexity -- this assumes all closed polygons
		std::shared_ptr<NavMesh> navMesh = _localizer->getNavMesh();
		const unsigned int OBST_COUNT = static_cast<unsigned int>(navMesh->getObstacleCount());
		for (unsigned int o = 0; o < OBST_COUNT; ++o) {
			NavMeshObstacle & obst = navMesh->GetObstacle(o);
			if (obst._prevObstacle) {
				obst._isConvex = MathUtil::leftOf(obst._prevObstacle->getP0(),
					obst.getP0(), obst.getP1()) >= 0;
			}
			else {
				obst._isConvex = true;
			}
		}
	}

	std::set<size_t> NavMeshSpatialQuery::ObstacleQuery(Vector2 pt) const
	{
		float range = 10.0f;
		return ObstacleQuery(pt, range);
	}

	std::set<size_t> NavMeshSpatialQuery::ObstacleQuery(Vector2 pt, float rangeSq) const
	{
		const size_t nodeId = _localizer->getNodeId(pt);
		if(nodeId == NavMeshLocation::NO_NODE)
			return std::set<size_t>();

		const NavMeshNode & node = _localizer->getNavMesh()->GetNode(nodeId);

		const size_t OBST_COUNT = node.getObstacleCount();
		std::set<size_t> result;
		for (size_t o = 0; o < OBST_COUNT; ++o) {
			const NavMeshObstacle * obst = node.getObstacle(o);
			if (obst->pointOutside(pt)) {
				float distance = distSqPointLineSegment(obst->getP0(), obst->getP1(), pt);

				if(distance < rangeSq) result.insert(obst->getId());
			}
		}

		return result;
	}
}
