#pragma once

#include "IStrategyComponent.h"
#include "Goal/Goal.h"
#include "Config.h"

#include <map>

namespace FusionCrowd
{
	namespace Strategy
	{
		class FUSION_CROWD_API TestStrategyComponent:
			public IStrategyComponent
		{
		public:
			void Update() {}

			void AddGoal(int idAgent, Goal* goal);
			Goal* GetGoal(int idAgent);

			~TestStrategyComponent() {};
		private:
			std::map<int, Goal*> infoGoalAgents;
		};
	}
}

