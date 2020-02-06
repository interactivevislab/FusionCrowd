#include "Group.h"

namespace  FusionCrowd
{
	Group::Group() : id(0), dummyAgentId(-1), _shape(nullptr)
	{ }

	Group::Group(size_t id, size_t dummyId, std::unique_ptr<IGroupShape> shape)
		: id(id), dummyAgentId(dummyId), _shape(std::move(shape))
	{ }

	IGroupShape* Group::GetShape() const
	{
		return _shape.get();
	}

	size_t Group::GetSize() const
	{
		return _agents.size();
	}

	bool Group::Contains(size_t agentId)
	{
		return _agents.find(agentId) != _agents.end();
	}

	void Group::AddAgent(size_t agentId)
	{
		if(_agents.find(agentId) != _agents.end())
			return;

		_agents.insert(agentId);
		_shape->AddAgent(agentId);
	}

	void Group::RemoveAgent(size_t agentId)
	{
		if(_agents.find(agentId) == _agents.end())
			return;

		_agents.erase(agentId);
		_shape->RemoveAgent(agentId);
	}

	std::vector<size_t> Group::GetAgents()
	{
		return std::vector<size_t>(_agents.begin(), _agents.end());
	}
}
