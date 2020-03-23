#pragma once

#include "Group/Group.h"

#include <unordered_set>

namespace FusionCrowd
{
	class LeaderGroup : public Group
	{
	public:
		explicit LeaderGroup(size_t leaderId);

		void AddAgent(size_t agentId, AgentSpatialInfo& info) override;
		void RemoveAgent(size_t agentId) override;
		DirectX::SimpleMath::Vector2 GetRelativePos(size_t agentId) const override;
		size_t GetSize() const override;
		bool Contains(size_t agentId) const override;
		std::vector<size_t> GetAgents() const override;

	private:
		size_t _leader;
		std::unordered_set<size_t> _followers;
	}
}
