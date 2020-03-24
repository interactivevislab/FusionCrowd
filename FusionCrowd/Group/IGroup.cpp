#include "IGroup.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	IGroup::IGroup(size_t id, size_t dummyId) : _id(id), _dummyAgentId(dummyId) { }

	void IGroup::SetAgentPrefVelocity(AgentSpatialInfo & dummyInfo, AgentSpatialInfo & agentInfo, float timeStep) const
	{
		float rot = atan2f(dummyInfo.orient.y, dummyInfo.orient.x);
		Vector2 relativePos = GetRelativePos(agentInfo.id);
		Vector2 rotRelativePos = MathUtil::rotate(relativePos, rot);
		Vector2 targetPos = dummyInfo.pos + rotRelativePos + dummyInfo.vel * timeStep;
		Vector2 dir = targetPos - agentInfo.pos;

		float speed = dir.Length() / timeStep;
		if(speed > agentInfo.maxSpeed)
		{
			speed = agentInfo.maxSpeed;
		}

		agentInfo.prefVelocity.setSpeed(speed);

		dir.Normalize();
		agentInfo.prefVelocity.setSingle(dir);
	}

	size_t IGroup::GetDummyId() const
	{
		return _dummyAgentId;
	}

	size_t IGroup::GetId() const
	{
		return _id;
	}
}
