#pragma once

#include <memory>
#include <vector>
#include <functional>

#include "Config.h"
#include "Math/Util.h"
#include "Util/PublicSpatialInfo.h"
#include "Util/IRecording.h"

namespace FusionCrowd
{
	class NavMeshComponent;
	class Obstacle;
	struct AgentSpatialInfo;

	class NavSystem
	{
	public:
		NavSystem(std::shared_ptr<NavMeshComponent> component);
		~NavSystem();

		NavSystem(NavSystem&&);
		NavSystem& operator=(NavSystem&&);

		FUSION_CROWD_API PublicSpatialInfo GetPublicSpatialInfo(size_t agentId);
		FUSION_CROWD_API int CountNeighbors(size_t agentId) const; //TEST METHOD, MUST BE DELETED
		FUSION_CROWD_API void SetAgentsSensitivityRadius(float radius);
		FUSION_CROWD_API IRecording & GetRecording();

		void AddAgent(size_t agentId, DirectX::SimpleMath::Vector2 position);
		void AddAgent(AgentSpatialInfo spatialInfo);

		AgentSpatialInfo & GetSpatialInfo(size_t agentId);

		const std::vector<AgentSpatialInfo> & GetNeighbours(size_t agentId) const;
		std::vector<Obstacle> GetClosestObstacles(size_t agentId) const;
		FUSION_CROWD_API void SetGridCoeff(float coeff);

		void Init();
		void Update(float timeStep);

	private:
		class NavSystemImpl;

		std::unique_ptr<NavSystemImpl> pimpl;
	};
}
