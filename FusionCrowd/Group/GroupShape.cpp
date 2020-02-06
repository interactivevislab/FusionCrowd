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

	void GroupGridShape::AddAgent(size_t agentId)
	{
		auto a = std::find(_agents.begin(), _agents.end(), agentId);

		if(a != _agents.end())
		{
			return;
		}

		_agents.push_back(agentId);
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

		float top   = AGENT_SIZE / 2.0f + row * (interAgentDist + AGENT_SIZE);
		float left  = AGENT_SIZE / 2.0f + pos * (interAgentDist + AGENT_SIZE);

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

		return AGENT_SIZE + (totalRows - 1) * (interAgentDist + AGENT_SIZE);
	}

	float GroupGridShape::GetTotalHeight() const
	{
		return AGENT_SIZE + (agentsInRow - 1) * (interAgentDist + AGENT_SIZE);
	}
}
