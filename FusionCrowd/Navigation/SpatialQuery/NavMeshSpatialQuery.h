#pragma once

#include "Agent.h"
#include "Math/Util.h"

#include "Navigation/Obstacle.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"
#include "Navigation/NavMesh/QuadTree.h"

#include <vector>

namespace FusionCrowd
{
	class NavMeshSpatialQuery
	{
	public:
		NavMeshSpatialQuery(std::shared_ptr<NavMeshLocalizer> nml);

		std::set<size_t> ObstacleQuery(DirectX::SimpleMath::Vector2 pt) const;
		std::set<size_t> ObstacleQuery(DirectX::SimpleMath::Vector2 pt, float rangeSq) const;
		bool QueryVisibility(
			const DirectX::SimpleMath::Vector2& q1,
			const DirectX::SimpleMath::Vector2& q2, float radius
		) const;

	private:
		void ProcessObstacles();

		std::unique_ptr<QuadTree> _obstacleBBTree;
		std::shared_ptr<NavMeshLocalizer> _localizer;
	};
}

