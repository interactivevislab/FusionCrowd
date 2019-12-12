#include "Agent.h"
#include "StrategyComponent/Goal/Goal.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	Agent::Agent(size_t agentId, Goal goal) : id(agentId), currentGoal(std::move(goal))
	{
	}
}
