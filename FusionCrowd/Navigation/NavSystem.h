#pragma once

#include <map>
#include <string>
#include <vector>

#include "Config.h"
#include "Math/Util.h"
#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/Obstacle.h"
#include "Navigation/NavMesh/NavMesh.h"
#include "Navigation/SpatialQuery/NavMeshSpatialQuery.h"

namespace FusionCrowd
{
	class NavMeshComponent;

	class FUSION_CROWD_API NavSystem
	{
	public:
		NavSystem(NavMeshComponent & component);
		~NavSystem();

		void AddAgent(size_t agentId, DirectX::SimpleMath::Vector2 position);
		void AddAgent(AgentSpatialInfo spatialInfo);
		AgentSpatialInfo & GetSpatialInfo(size_t agentId);

		std::vector<AgentSpatialInfo> GetNeighbours(size_t agentId) const;
		std::vector<Obstacle> GetClosestObstacles(size_t agentId) const;

		void Update(float timeStep);

	private:
		void UpdatePos(AgentSpatialInfo & agent, float timeStep);
		void UpdateOrient(AgentSpatialInfo & agent, float timeStep);

		std::map<size_t, AgentSpatialInfo> _agentSpatialInfos;
		NavMeshSpatialQuery _navMeshQuery;
		std::shared_ptr<NavMesh> _navMesh;
	};
}
