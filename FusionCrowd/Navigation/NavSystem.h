#pragma once

#include <memory>
#include <vector>

#include "Math/Util.h"
#include "Config.h"


namespace FusionCrowd
{
	class NavMeshComponent;
	class Obstacle;
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
		NavSystem(std::shared_ptr<NavMeshComponent> component);
		~NavSystem();

		NavSystem(NavSystem&&);
		NavSystem& operator=(NavSystem&&);

		FUSION_CROWD_API PublicSpatialInfo GetPublicSpatialInfo(size_t agentId);
		FUSION_CROWD_API int CountNeighbors(size_t agentId) const;								//TEST METHOD, MUST BE DELETED
		FUSION_CROWD_API void SetAgentsSensitivityRadius(float radius);

		void AddAgent(size_t agentId, DirectX::SimpleMath::Vector2 position);
		void AddAgent(AgentSpatialInfo spatialInfo);

		AgentSpatialInfo & GetSpatialInfo(size_t agentId);

		std::vector<AgentSpatialInfo> GetNeighbours(size_t agentId) const;
		std::vector<Obstacle> GetClosestObstacles(size_t agentId) const;

		void Update(float timeStep);

	private:
		class NavSystemImpl;

		std::unique_ptr<NavSystemImpl> pimpl;
	};
}
