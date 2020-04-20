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
				obst._isConvex = Math::leftOf(obst._prevObstacle->getP0(), obst.getP0(), obst.getP1()) >= 0;
			}
			else {
				obst._isConvex = true;
			}

			boxes.push_back({ obst.GetBB(), obst.getId()});
		}

		_obstacleBBTree = std::make_unique<QuadTree>(boxes);
	}

	Vector2 NavMeshSpatialQuery::GetClosiestObstacle(BoundingBox bb) {
		float bb_size = 1000.0f;
		bb.xmax += bb_size;
		bb.ymax += bb_size;
		bb.ymin -= bb_size;
		bb.xmin -= bb_size;
		auto mesh = _localizer->getNavMesh();

		Vector2 center = Vector2((bb.xmax + bb.xmin) / 2, (bb.ymax + bb.ymin) / 2);
		auto obstacleIds = _obstacleBBTree->GetIntersectingBBIds(bb);
		float max_dist = INFINITY;
		NavMeshObstacle* res_obst = nullptr;
		//for (size_t obstacleId : obstacleIds)
		for (size_t obstacleId = 0; obstacleId< _localizer->getNavMesh()->getObstacleCount(); obstacleId++)
		{
			NavMeshObstacle & obst = mesh->GetObstacle(obstacleId);
			float dist = obst.distSqPoint(center);
			if (dist < max_dist) {
				dist = max_dist;
				res_obst = &obst;
			}
		}
		Vector2 res = res_obst->midPt() - center;
		res.Normalize();
		res = res * 0.3f + res_obst->midPt();
		return res;
	}

	std::vector<size_t> NavMeshSpatialQuery::ObstacleQuery(Vector2 pt) const
	{
		float range = 3.2f;
		return ObstacleQuery(pt, range);
	}

	std::vector<size_t> NavMeshSpatialQuery::ObstacleQuery(Vector2 pt, float range) const
	{
		const float rangeSq = range * range;
		auto bb = BoundingBox(pt.x - range, pt.y - range, pt.x + range, pt.y + range);
		auto obstacleIds = _obstacleBBTree->GetIntersectingBBIds(bb);
		auto mesh = _localizer->getNavMesh();
		std::vector<size_t> result;

		//for(size_t obstacleId = 0; obstacleId < mesh->getObstacleCount(); obstacleId++)
		for(size_t obstacleId : obstacleIds)
		{
			const NavMeshObstacle & obst = mesh->GetObstacle(obstacleId);

			if (obst.pointOutside(pt))
			{
				float distance = obst.distSqPoint(pt);
				if(distance < rangeSq)
				{
					result.push_back(obst.getId());
				}
			}
		}

		return result;
	}

	void NavMeshSpatialQuery::Update() {
		ProcessObstacles();
	}
}
