#include "GuidedGroup.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	GuidedGroup::GuidedGroup(size_t id, size_t dummyId, const AgentSpatialInfo& leader)
		: IGroup(id, dummyId), _leader(leader.id), _maxRadius(leader.radius)
	{}

	void GuidedGroup::AddAgent(size_t agentId, AgentSpatialInfo& info)
	{
		if(_leader == agentId || _followers.find(agentId) != _followers.end())
		{
			return;
		}

		_followers.insert(agentId);
	}

	void GuidedGroup::RemoveAgent(size_t agentId)
	{
		_followers.erase(agentId);
	}

	DirectX::SimpleMath::Vector2 GuidedGroup::GetRelativePos(size_t agentId) const
	{
		return Vector2();
	}

	size_t GuidedGroup::GetSize() const
	{
		return _followers.size() + 1;
	}

	bool GuidedGroup::Contains(size_t agentId) const
	{
		return _followers.find(agentId) != _followers.end();
	}

	std::vector<size_t> GuidedGroup::GetAgents() const
	{
		return std::vector<size_t>(_followers.begin(), _followers.end());
	}

	float GuidedGroup::GetRadius() const
	{
		return _maxRadius;
	}
}
