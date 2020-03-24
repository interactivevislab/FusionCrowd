#pragma once

#include "Group/IGroup.h"
#include "Math/Util.h"

#include <unordered_set>

namespace FusionCrowd
{
	class GuidedGroup : public IGroup
	{
	public:
		GuidedGroup(size_t id, size_t dummyId, size_t leaderId);
		
		void AddAgent(size_t agentId, AgentSpatialInfo& info) override;
		void RemoveAgent(size_t agentId) override;
		DirectX::SimpleMath::Vector2 GetRelativePos(size_t agentId) const override;
		size_t GetSize() const override;
		bool Contains(size_t agentId) const override;
		std::vector<size_t> GetAgents() const override;
		float GetRadius() const override;

	private:
		size_t _leader;
		std::unordered_set<size_t> _followers;
	};
}
