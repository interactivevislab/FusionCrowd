#pragma once

#include "Math/Util.h"
#include "Navigation/AgentSpatialInfo.h"
#include <vector>

namespace FusionCrowd
{
	class IGroupShape
	{
	public:
		virtual void AddAgent(size_t agentId, AgentSpatialInfo& info) = 0;
		virtual void RemoveAgent(size_t agentId) = 0;
		virtual DirectX::SimpleMath::Vector2 GetRelativePos(size_t agentId) const = 0;
		virtual size_t GetSize() const = 0;
		virtual float GetRadius() const = 0;
	};

	class GroupGridShape : public IGroupShape
	{
	public:
		GroupGridShape(size_t agentsInRow, float interAgentDist);

		const size_t agentsInRow;
		const float interAgentDist;

		void AddAgent(size_t agentId, AgentSpatialInfo& info) override;
		void RemoveAgent(size_t agentId) override;
		DirectX::SimpleMath::Vector2 GetRelativePos(size_t agentId) const override;
		size_t GetSize() const override;
		float GetRadius() const override;

	private:
		float GetTotalWidth() const;
		float GetTotalHeight() const;

		std::vector<size_t> _agents;

		float _agentSize = 0.0f;
	};
}
