#include "NavMeshSpatialQuery.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	NavMeshSpatialQuery::NavMeshSpatialQuery(std::shared_ptr<NavMeshLocalizer> nml) :
		_localizer(nml)
	{
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
		std::vector<QuadTree::Box> boxes;

		const unsigned int OBST_COUNT = static_cast<unsigned int>(navMesh->getObstacleCount());
		for (unsigned int o = 0; o < OBST_COUNT; ++o) {
			NavMeshObstacle & obst = navMesh->GetObstacle(o);
			if (obst._prevObstacle) {
				obst._isConvex = MathUtil::leftOf(obst._prevObstacle->getP0(), obst.getP0(), obst.getP1()) >= 0;
			}
			else {
				obst._isConvex = true;
			}

			boxes.push_back({ obst.GetBB(), obst.getId()});
		}

		_obstacleBBTree = std::make_unique<QuadTree>(boxes);
	}


	std::set<size_t> NavMeshSpatialQuery::ObstacleQuery(Vector2 pt) const
	{
		float range = 3.2f;
		return ObstacleQuery(pt, range);
	}

	std::set<size_t> NavMeshSpatialQuery::ObstacleQuery(Vector2 pt, float range) const
	{
		const float rangeSq = range * range;
		auto obstacleIds = _obstacleBBTree->GetIntersectingBBIds(BoundingBox(pt.x - range, pt.y - range, pt.x + range, pt.y + range));
		auto mesh = _localizer->getNavMesh();
		std::set<size_t> result;

		//for(size_t obstacleId = 0; obstacleId < mesh->getObstacleCount(); obstacleId++)
		for(size_t obstacleId : obstacleIds)
		{
			const NavMeshObstacle & obst = mesh->GetObstacle(obstacleId);
			if (obst.pointOutside(pt))
			{
				float distance = obst.distSqPoint(pt);
				if(distance < rangeSq)
				{
					result.insert(obst.getId());
				}
			}
		}

		return result;
	}
}
