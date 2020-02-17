#include "GroupShape.h"
#include "Math/Util.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	GroupGridShape::GroupGridShape(size_t agentsInRow, float interAgentDist)
		: agentsInRow(agentsInRow), interAgentDist(interAgentDist)
	{
	}

	size_t GroupGridShape::GetSize() const
	{
		return _agents.size();
	}

	void GroupGridShape::AddAgent(size_t agentId, AgentSpatialInfo& info)
	{
		auto a = std::find(_agents.begin(), _agents.end(), agentId);

		if(a != _agents.end())
		{
			return;
		}

		_agents.push_back(agentId);

		if(info.radius * 2 > _agentSize)
			_agentSize = info.radius * 2;
	}

	void GroupGridShape::RemoveAgent(size_t agentId)
	{
		auto a = std::find(_agents.begin(), _agents.end(), agentId);

		if(a == _agents.end())
		{
			return;
		}

		_agents.erase(a);
	}

	Vector2 GroupGridShape::GetRelativePos(size_t agentId) const
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

		return Vector2(totalWidth / 2.0f - left, -totalHeight / 2.0f + top);
	}

	float GroupGridShape::GetRadius() const
	{
		float totalHeight = GetTotalHeight();
		float totalWidth  = GetTotalWidth();

		return sqrtf(totalHeight * totalHeight + totalWidth * totalWidth) / 2.0f;
	}

	float GroupGridShape::GetTotalWidth() const
	{
		size_t totalRows = _agents.size() / agentsInRow;
		if(_agents.size() % agentsInRow > 0)
			totalRows++;

		return _agentSize + (totalRows - 1) * (interAgentDist + _agentSize);
	}

	float GroupGridShape::GetTotalHeight() const
	{
		return _agentSize + (agentsInRow - 1) * (interAgentDist + _agentSize);
	}
}
