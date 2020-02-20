#pragma once

#include "Group/IGroupShape.h"
#include "Math/Util.h"

#include <vector>
#include <random>
#include <map>

namespace FusionCrowd
{
	class FreeGridShape : public IGroupShape
	{
	public:
		FreeGridShape(size_t agentsInRow, float interAgentDist);

		const size_t agentsInRow;
		const float interAgentDist;

		void AddAgent(size_t agentId, AgentSpatialInfo& info) override;
		void RemoveAgent(size_t agentId) override;
		DirectX::SimpleMath::Vector2 GetRelativePos(size_t agentId) const override;
		size_t GetSize() const override;
		float GetRadius() const override;

	private:
		std::vector<size_t> _agents;
		std::map<size_t, DirectX::SimpleMath::Vector2> _random_shift;

		float GetTotalWidth() const;
		float GetTotalHeight() const;

		float _agentSize = 0.0f;

		std::random_device _rnd_device;
		std::mt19937 _rnd_seed;
	};
}