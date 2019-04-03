#pragma once
#include "../../Config.h"
#include "../../NavComponents/Obstacle.h"
#include "ProximityQuery.h"

#include <vector>

namespace FusionCrowd
{
	// FORWARD DECLARATIONS
	class Agent;

	class FUSION_CROWD_API SpatialQuery
	{
	public:
		SpatialQuery();
		~SpatialQuery() {}

		virtual void SetAgents(const std::vector< Agent * > & agents) = 0;
		virtual void UpdateAgents() = 0;
		virtual void AddObstacle(Obstacle * obs);
		const std::vector< Obstacle * > & GetObstacles() const { return _obstacles; }
		virtual void AgentQuery(ProximityQuery * query) const = 0;
		virtual void ProcessObstacles() = 0;
		virtual void ObstacleQuery(ProximityQuery * query) const = 0;
		virtual bool QueryVisibility(const Math::Vector2 & q1, const Math::Vector2 & q2,
			float radius) const = 0;
		virtual void SetNeighborVisibleTest(bool state) {}
		inline void SetTestVisibility(bool status) { _testVisibility = status; }
		inline bool GetTestVisibility() const { return _testVisibility; }

		bool _testVisibility;
		std::vector<Obstacle *> _obstacles;
	};
}
