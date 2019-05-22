#pragma once

#include <map>
#include <string>
#include <vector>

#include "Config.h"
#include "Math/Util.h"
#include "Navigation/INavComponent.h"
#include "Navigation/Obstacle.h"
#include "Navigation/AgentSpatialInfo.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API NavSystem
	{
	public:
		NavSystem();
		~NavSystem();

		void AddNavComponent(std::string name, INavComponent navComponent);
		INavComponent & GetNavComponent(std::string name);

		void AddAgent(size_t agentId, DirectX::SimpleMath::Vector2 position);
		void AddAgent(size_t agentId, AgentSpatialInfo spatialInfo);
		AgentSpatialInfo & GetSpatialInfo(size_t agentId);

		std::vector<AgentSpatialInfo> GetNeighbours(size_t agentId) const;
		std::vector<Obstacle> GetClosestObstacles(size_t agentId) const;

		void Update(float timeStep);

	private:
		void UpdatePos(AgentSpatialInfo & agent, float timeStep);
		void UpdateOrient(AgentSpatialInfo & agent, float timeStep);

		std::map<std::string, INavComponent> _navComponents;
		std::map<size_t, AgentSpatialInfo> _agentSpatialInfos;
	};
}
