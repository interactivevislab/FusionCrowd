#pragma once

#include "Group/IGroup.h"

#include <vector>

namespace FusionCrowd
{
	class GridGroup : public IGroup
	{
	public:
		GridGroup(size_t agentsInRow, float interAgentDist);

		const size_t agentsInRow;
		const float interAgentDist;

		void AddAgent(size_t agentId, AgentSpatialInfo& info) override;
		void RemoveAgent(size_t agentId) override;
		DirectX::SimpleMath::Vector2 GetRelativePos(size_t agentId) const override;
		size_t GetSize() const override;

	private:
		float GetRadius() const;
		float GetTotalWidth() const;
		float GetTotalHeight() const;

		std::vector<size_t> _agents;

		float _agentSize = 0.0f;
	};
}