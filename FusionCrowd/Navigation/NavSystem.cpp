#include "NavSystem.h"
#include "Math/Util.h"
#include "TacticComponent/NavMeshComponent.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	NavSystem::NavSystem(std::shared_ptr<NavMeshComponent> component) :
		_navMeshQuery(NavMeshSpatialQuery(component->GetLocalizer()))
	{
		_navMesh = component->GetNavMesh();
	}

	void NavSystem::AddAgent(size_t agentId, Vector2 position)
	{
		AgentSpatialInfo info;
		info.id = agentId;
		info.pos = position;

		_agentSpatialInfos.insert({agentId, info});
	}

	void NavSystem::AddAgent(AgentSpatialInfo spatialInfo)
	{
		_agentSpatialInfos.insert({spatialInfo.id, spatialInfo});
	}

	AgentSpatialInfo & NavSystem::GetSpatialInfo(size_t agentId)
	{
		return _agentSpatialInfos[agentId];
	}

	std::vector<AgentSpatialInfo> NavSystem::GetNeighbours(size_t agentId) const
	{
		std::vector<AgentSpatialInfo> result;
		for(auto & pair : _agentSpatialInfos)
			if(pair.first != agentId)
				result.push_back(pair.second);

		return result;
	}

	std::vector<Obstacle> NavSystem::GetClosestObstacles(size_t agentId) const
	{
		AgentSpatialInfo agent = _agentSpatialInfos.at(agentId);

		std::vector<Obstacle> result;
		for(size_t obstId : _navMeshQuery.ObstacleQuery(agent.pos))
		{
			result.push_back(_navMesh->GetObstacle(obstId));
		}

		return result;
	}

	void NavSystem::Update(float timeStep)
	{
		for (auto & pair : _agentSpatialInfos)
		{
			size_t id = pair.first;
			AgentSpatialInfo & info = pair.second;

			UpdatePos(info, timeStep);
			UpdateOrient(info, timeStep);
		}
	}

	void NavSystem::UpdatePos(AgentSpatialInfo & agent, float timeStep)
	{
		float delV = (agent.vel - agent.velNew).Length();

		if (delV > agent.maxAccel * timeStep) {
			float w = agent.maxAccel * timeStep / delV;
			agent.vel = (1.f - w) * agent.vel + w * agent.velNew;
		}
		else {
			agent.vel = agent.velNew;
		}

		agent.pos += agent.vel * timeStep;
	}

	void NavSystem::UpdateOrient(AgentSpatialInfo & agent, float timeStep)
	{
		float speed = agent.vel.Length();
		const float speedThresh = agent.prefSpeed / 3.f;
		Vector2 newOrient(agent.orient); // by default new is old
		Vector2 moveDir = agent.vel / speed;
		if (speed >= speedThresh)
		{
			newOrient = moveDir;
		}
		else
		{
			float frac = sqrtf(speed / speedThresh);
			Vector2 prefDir = agent.prefVelocity.getPreferred();
			// prefDir *can* be zero if we've arrived at goal.  Only use it if it's non-zero.
			if (prefDir.LengthSquared() > 0.000001f)
			{
				newOrient = frac * moveDir + (1.f - frac) * prefDir;
				newOrient.Normalize();
			}
		}

		// Now limit angular velocity.
		const float MAX_ANGLE_CHANGE = timeStep * agent.maxAngVel;
		float maxCt = cos(MAX_ANGLE_CHANGE);
		float ct = newOrient.Dot(agent.orient);
		if (ct < maxCt)
		{
			// changing direction at a rate greater than _maxAngVel
			float maxSt = sin(MAX_ANGLE_CHANGE);
			if (MathUtil::det(agent.orient, newOrient) > 0.f)
			{
				// rotate _orient left
				agent.orient = Vector2(
					maxCt * agent.orient.x - maxSt * agent.orient.y,
					maxSt * agent.orient.x + maxCt * agent.orient.y
				);
			}
			else
			{
				// rotate _orient right
				agent.orient = Vector2(
					maxCt * agent.orient.x + maxSt * agent.orient.y,
					-maxSt * agent.orient.x + maxCt * agent.orient.y
				);
			}
		}
		else
		{
			agent.orient = newOrient;
		}
	}

	NavSystem::~NavSystem()
	{
	}
}
