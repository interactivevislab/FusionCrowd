#pragma once

#include "Config.h"
#include "Agent.h"
#include "Math/Util.h"

#include "Navigation/Obstacle.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"

#include <vector>

namespace FusionCrowd
{
	class FUSION_CROWD_API NavMeshSpatialQuery
	{
	public:
		NavMeshSpatialQuery(std::shared_ptr<NavMeshLocalizer> nml);

		void ProcessObstacles();
		std::set<size_t> ObstacleQuery(Vector2 pt) const;
		std::set<size_t> ObstacleQuery(Vector2 pt, float rangeSq) const;
		bool QueryVisibility(
			const DirectX::SimpleMath::Vector2& q1,
			const DirectX::SimpleMath::Vector2& q2, float radius
		) const;

		std::shared_ptr<NavMeshLocalizer> _localizer;
	};
}

