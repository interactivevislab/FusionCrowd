#include "GridGroup.h"
#include "Math/Util.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	GridGroup::GridGroup(size_t id, size_t dummyId, size_t agentsInRow, float interAgentDist)
		: IGroup(id, dummyId), agentsInRow(agentsInRow), interAgentDist(interAgentDist)
	{
	}

	size_t GridGroup::GetSize() const
	{
		return _agents.size();
	}

	void GridGroup::AddAgent(size_t agentId, AgentSpatialInfo& info)
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

	void GridGroup::RemoveAgent(size_t agentId)
	{
		auto a = std::find(_agents.begin(), _agents.end(), agentId);

		if(a == _agents.end())
		{
			return;
		}

		_agents.erase(a);
	}

	Vector2 GridGroup::GetRelativePos(size_t agentId) const
	{
		auto a = std::find(_agents.begin(), _agents.end(), agentId);
		if(a == _agents.end())
		{
			// agent not found, return center
			return Vector2();
		}

		size_t agtIndex = std::distance(_agents.begin(), a);
		size_t pos = agtIndex % agentsInRow;
		size_t row = agtIndex / agentsInRow;

		float totalWidth  = GetTotalWidth();
		float totalHeight = GetTotalHeight();

		float paddingRight = _agentSize / 2.f + row * (interAgentDist + _agentSize);
		float paddingTop   = _agentSize / 2.f + pos * (interAgentDist + _agentSize);

		return Vector2(totalWidth / 2.0f - paddingRight, totalHeight / 2.0f - paddingTop);
	}

	float GridGroup::GetRadius() const
	{
		float totalHeight = GetTotalHeight();
		float totalWidth  = GetTotalWidth();

		return sqrtf(totalHeight * totalHeight + totalWidth * totalWidth) / 2.0f;
	}

	float GridGroup::GetTotalHeight() const
	{
		return _agentSize + (agentsInRow - 1) * (interAgentDist + _agentSize);
	}

	float GridGroup::GetTotalWidth() const
	{
		if(_agents.size() == 0)
		{
			return 0.0f;
		}

		size_t totalRows = _agents.size() / agentsInRow;
		if(_agents.size() % agentsInRow != 0)
		{
			totalRows++;
		}

		return _agentSize + (totalRows - 1) * (interAgentDist + _agentSize);
	}

	bool GridGroup::Contains(size_t agentId) const
	{
		return std::find(_agents.begin(), _agents.end(), agentId) == _agents.end();
	}

	std::vector<size_t> GridGroup::GetAgents() const
	{
		return std::vector<size_t>(_agents);
	}
}
