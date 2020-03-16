#include "Group.h"

#include "Simulator.h"
#include "Math/Util.h"

using namespace DirectX::SimpleMath;

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

	void Group::AddAgent(size_t agentId, AgentSpatialInfo& info)
	{
		if(_agents.find(agentId) != _agents.end())
			return;

		_agents.insert(agentId);
		info.inertiaEnabled = false;
		_shape->AddAgent(agentId, info);
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

	void Group::SetAgentPrefVelocity(AgentSpatialInfo & dummyInfo, AgentSpatialInfo & agentInfo, float timeStep) const
	{
		float rot = atan2f(dummyInfo.orient.y, dummyInfo.orient.x);
		Vector2 relativePos = MathUtil::rotate(GetShape()->GetRelativePos(agentInfo.id), rot);
		Vector2 targetPos = dummyInfo.pos + relativePos;
		Vector2 dir = targetPos - agentInfo.pos;

		float speed = dir.Length() * timeStep;
		if(agentInfo.prefSpeed < speed)
		{
			speed = agentInfo.prefSpeed;
		}
		agentInfo.prefVelocity.setSpeed(dir.Length());

		dir.Normalize();
		agentInfo.prefVelocity.setSingle(dir);
	}
}
