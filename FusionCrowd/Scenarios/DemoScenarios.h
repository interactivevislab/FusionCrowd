#pragma once

#include "../Simulator.h"
#include "../StrategyComponent/Goal/Goal.h"
#include "../Config.h"

#include <vector>


namespace FusionCrowd
{
	namespace Scenarios
	{
		static class FUSION_CROWD_API DemoScenarios
		{
		public:
			static void RunSochiDemoScenarioInitialize(Simulator* sim,const char* pathNavMesh, int agentCount);

			static void AddSpatialQuery(Simulator* sim, FusionCrowd::SpatialQuery* spatialQuery);
		private:
			static std::vector<DirectX::SimpleMath::Vector2> GetPositionAgent(int agentCount);
		};
	}
}


