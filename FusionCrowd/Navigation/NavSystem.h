#pragma once

#include <map>
#include <string>
#include "Config.h"

#include "Agent.h"
#include "Navigation/INavComponent.h"
#include "Navigation/SpatialQuery/SpatialQuery.h"
#include "Math/Util.h"
#include <vector>

namespace FusionCrowd
{
	struct FUSION_CROWD_API AgentSpatialInfo
	{
		DirectX::SimpleMath::Vector2 orientation;
		DirectX::SimpleMath::Vector2 position;
		DirectX::SimpleMath::Vector2 velocity;
		float radius;
	};

	class FUSION_CROWD_API NavSystem
	{
	public:
		NavSystem();
		~NavSystem();

		void AddNavComponent(std::string name, INavComponent* navComponent);
		INavComponent* GetNavComponent(std::string name);

		void AddAgent(AgentSpatialInfo spatialInfo, INavComponent* navComponent);

		void AddSpatialQuery(FusionCrowd::SpatialQuery* spatialQuery);
		void ComputeNeighbors(FusionCrowd::Agent* agent);

		void Update(float timeStep);
	private:
		std::map<std::string, INavComponent *> _navComponents;
		std::vector<AgentSpatialInfo> _agentSpatialInfos;
	};
}
