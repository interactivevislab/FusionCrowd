#include "Agent.h"

#include "StrategyComponent/Goal/Goal.h"
#include "Group/IGroup.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	Agent::Agent(size_t agentId, Goal goal) : id(agentId), currentGoal(std::move(goal)), _groupId(IGroup::NO_GROUP)
	{
	}

	size_t Agent::GetGroupId() const
	{
		return _groupId;
	}

	void Agent::SetGroupId(size_t newGroupId)
	{
		_groupId = newGroupId;
	}
}
