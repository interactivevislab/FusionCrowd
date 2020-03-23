#include "LeaderGroup.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	LeaderGroup::LeaderGroup(size_t leaderId) : _leader(leaderId)
	{}

	void LeaderGroup::AddAgent(size_t agentId, AgentSpatialInfo& info)
	{
		if(_leader == agentId || _followers.find(agentId) != _followers.end())
		{
			return;
		}

		_followers.insert(agentId);
	}

	void LeaderGroup::RemoveAgent(size_t agentId)
	{
		_followers.erase(agentId);
	}

	DirectX::SimpleMath::Vector2 LeaderGroup::GetRelativePos(size_t agentId) const
	{
		if(agentId == _leader || _followers.find(agentId) == _followers.end())
			return Vector2();


	}

	size_t LeaderGroup::GetSize() const
	{
	}
}
