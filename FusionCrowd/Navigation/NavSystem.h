#pragma once

#include <map>
#include <unordered_map>
#include <string>
#include <vector>
#include <functional>

#include "Config.h"
#include "Math/Util.h"
#include "Navigation/Obstacle.h"
#include "Navigation/NavMesh/NavMesh.h"
#include "Navigation/SpatialQuery/NavMeshSpatialQuery.h"
#include "Navigation/FastFixedRadiusNearestNeighbors/NeighborsSeeker.h"

namespace FusionCrowd
{
	class NavMeshComponent;
	struct AgentSpatialInfo;

	struct FUSION_CROWD_API PublicSpatialInfo
	{
		size_t id;
		float posX, posY;
		float velX, velY;
		float orientX, orientY;
		float radius;
	};

	class NavSystem
	{
	public:
		FUSION_CROWD_API NavSystem(std::shared_ptr<NavMeshComponent> component);
		FUSION_CROWD_API ~NavSystem();
		FUSION_CROWD_API PublicSpatialInfo GetPublicSpatialInfo(size_t agentId);

		void AddAgent(size_t agentId, DirectX::SimpleMath::Vector2 position);
		void AddAgent(AgentSpatialInfo spatialInfo);
		AgentSpatialInfo & GetSpatialInfo(size_t agentId);

		const std::vector<AgentSpatialInfo> & GetNeighbours(size_t agentId) const;
		FUSION_CROWD_API int CountNeighbors(size_t agentId) const;								//TEST METHOD, MUST BE DELETED
		std::vector<Obstacle> GetClosestObstacles(size_t agentId) const;
		FUSION_CROWD_API void SetGridCoeff(float coeff);

		void Init();
		void Update(float timeStep);

		FUSION_CROWD_API void SetAgentsSensitivityRadius(float radius);

	private:
		void UpdatePos(AgentSpatialInfo & agent, float timeStep);
		void UpdateOrient(AgentSpatialInfo & agent, float timeStep);
		void UpdateNeighbours();

		std::map<size_t, AgentSpatialInfo> _agentSpatialInfos;
		NavMeshSpatialQuery _navMeshQuery;
		std::shared_ptr<NavMesh> _navMesh;

		NeighborsSeeker _neighborsSeeker;
		std::unordered_map<size_t, std::vector<AgentSpatialInfo>> _agentsNeighbours;
		float _agentsSensitivityRadius = 1;
	};
}
