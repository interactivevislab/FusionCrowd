#include "GuidedGroup.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	GuidedGroup::GuidedGroup(size_t id, size_t dummyId, size_t leaderId) : IGroup(id, dummyId), _leader(leaderId)
	{}

	void GuidedGroup::AddAgent(size_t agentId, AgentSpatialInfo& info)
	{		
		if(_leader == agentId || _followers.find(agentId) != _followers.end())
		{
			return;
		}

		info.inertiaEnabled = false;

		_followers.insert(agentId);
	}

	void GuidedGroup::RemoveAgent(size_t agentId)
	{
		_followers.erase(agentId);
	}

	DirectX::SimpleMath::Vector2 GuidedGroup::GetRelativePos(size_t agentId) const
	{
		if(agentId == _leader || _followers.find(agentId) == _followers.end())
			return Vector2();

		throw "Not implemented";
		return Vector2();
	}

	size_t GuidedGroup::GetSize() const
	{
		throw "Not implemented";
		return 0;
	}


	bool GuidedGroup::Contains(size_t agentId) const
	{
		throw "Not implemented";
		return false;
	}

	std::vector<size_t> GuidedGroup::GetAgents() const
	{
		throw "Not implemented";
		return std::vector<size_t>();
	}

	float GuidedGroup::GetRadius() const
	{
		throw "Not implemented";
		return 0;
	}
}
