#include "TestStrategyComponent.h"

namespace FusionCrowd
{
	namespace Strategy
	{
		void TestStrategyComponent::AddGoal(int idAgent, Goal* goal)
		{
			infoGoalAgents[idAgent] = goal;
		}

		Goal* TestStrategyComponent::GetGoal(int idAgent)
		{
			if (infoGoalAgents.find(idAgent) != infoGoalAgents.end()){
				return infoGoalAgents[idAgent];
			}
			else{
				return NULL;
			}
		}
	}
}
