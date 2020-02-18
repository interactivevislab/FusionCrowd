#include "FreeGridShape.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	FreeGridShape::FreeGridShape(size_t agentsInRow, float interAgentDist)
		: agentsInRow(agentsInRow), interAgentDist(interAgentDist), _rnd_seed(_rnd_device())
	{ }

	void FreeGridShape::AddAgent(size_t agentId, AgentSpatialInfo& info)
	{
		if(std::find(_agents.begin(), _agents.end(), agentId) != _agents.end())
		{
			return;
		}

		_agents.push_back(agentId);
		if(info.radius > _agentSize)
		{
			_agentSize = info.radius;
		}

		std::uniform_real_distribution<float> x_dist(-_agentSize / 2.0f, _agentSize / 2.0f);
		std::uniform_real_distribution<float> y_dist(0.f, _agentSize / 2.0f);

		_random_shift.insert({agentId, Vector2(x_dist(_rnd_seed), y_dist(_rnd_seed))});
	}

	void FreeGridShape::RemoveAgent(size_t agentId)
	{
		auto agt = std::find(_agents.begin(), _agents.end(), agentId);
		if(agt == _agents.end())
		{
			return;
		}

		_agents.erase(agt);
	}

	Vector2 FreeGridShape::GetRelativePos(size_t agentId) const
	{
		auto a = std::find(_agents.begin(), _agents.end(), agentId);
		if(a == _agents.end())
		{
			// agent not found, return center
			return Vector2();
		}

		size_t agtIndex  = std::distance(_agents.begin(), a);
		size_t pos    = agtIndex / agentsInRow;
		size_t row = agtIndex % agentsInRow;

		float totalWidth  = GetTotalWidth();
		float totalHeight = GetTotalHeight();

		float top   = _agentSize / 2.0f + row * (interAgentDist + _agentSize);
		float left  = _agentSize / 2.0f + pos * (interAgentDist + _agentSize);


		return Vector2(totalWidth / 2.0f - left, -totalHeight / 2.0f + top) + _random_shift.find(agentId)->second;
	}

	size_t FreeGridShape::GetSize() const
	{
		return _agents.size();
	}

	float FreeGridShape::GetRadius() const
	{
		float totalHeight = GetTotalHeight();
		float totalWidth  = GetTotalWidth();

		return sqrtf(totalHeight * totalHeight + totalWidth * totalWidth) / 2.0f;
	}

	float FreeGridShape::GetTotalWidth() const
	{
		size_t totalRows = _agents.size() / agentsInRow;
		if(_agents.size() % agentsInRow > 0)
			totalRows++;

		return _agentSize + (totalRows - 1) * (interAgentDist + _agentSize);
	}

	float FreeGridShape::GetTotalHeight() const
	{
		return _agentSize + (agentsInRow - 1) * (interAgentDist + _agentSize);
	}
}
