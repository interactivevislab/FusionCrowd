#include "NavGraphComponent.h"
#include "Navigation/AgentSpatialInfo.h"


using namespace DirectX::SimpleMath;

namespace FusionCrowd
{

	NavGraphComponent::NavGraphComponent(std::shared_ptr<Simulator> simulator, std::shared_ptr<NavSystem> navSystem)
		: _simulator(simulator), _navSystem(navSystem), _navGraph(navSystem->GetNavGraph()), _pathPlanner(_navGraph)
	{ }

	void NavGraphComponent::AddAgent(size_t id)
	{
		size_t curNodeId = getNodeId(id);
		NavGraphPathPlanner pathPlanner(_navGraph);

		Vector2 curPos = _navSystem->GetSpatialInfo(id).GetPos();

		AgentStruct agtStruct;
		agtStruct.id = id;
		agtStruct.route = pathPlanner.GetRoute(curPos, curPos);
		agtStruct.pointsComplete = 0;
		_agents.push_back(agtStruct);
	}

	void NavGraphComponent::Replan(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct)
	{
		auto & agentGoal = _simulator->GetAgentGoal(agentStruct.id);

		agentStruct.route = _pathPlanner.GetRoute(agentInfo.GetPos(), agentGoal.getCentroid());

		agentStruct.pointsComplete = 0;
		agentStruct.goalPoint = agentGoal.getCentroid();
	}

	bool NavGraphComponent::DeleteAgent(size_t id)
	{
		for (int i = 0; i < _agents.size(); i++) {
			if (_agents[i].id == id) {
				_agents.erase(_agents.begin() + i);
				return true;
			}
		}
		return false;
	}

	void NavGraphComponent::Update(float timeStep)
	{
		for (auto & agtStruct : _agents)
		{
			size_t id = agtStruct.id;

			size_t groupId = _simulator->GetAgent(id).GetGroupId();
			AgentSpatialInfo & info = _simulator->GetSpatialInfo(id);

			if(groupId != IGroup::NO_GROUP)
			{
				auto grp = _simulator->GetGroup(groupId);
				auto & dummy = _simulator->GetSpatialInfo(grp->GetDummyId());
				if (grp == nullptr) {
					info.prefVelocity.setSpeed(0);
					continue;
				}

				grp->SetAgentPrefVelocity(dummy, info, timeStep);
				continue;
			}

			if (IsReplanNeeded(info, agtStruct))
			{
				Replan(info, agtStruct);
			}

			UpdateLocation(info, agtStruct, timeStep);
			SetPrefVelocity(info, agtStruct, timeStep);
		}
	}

	bool NavGraphComponent::IsReplanNeeded(AgentSpatialInfo& agentInfo, AgentStruct& agentStruct)
	{
		auto & agentGoal = _simulator->GetAgentGoal(agentInfo.id);

		return Vector2::DistanceSquared(agentStruct.goalPoint, agentGoal.getCentroid()) > 1e-6f;
	}

	void NavGraphComponent::SetPrefVelocity(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct, float timeStep)
	{
		Vector2 currentGoal = agentStruct.route.points[agentStruct.pointsComplete];

		float dist = Vector2::Distance(currentGoal, agentInfo.GetPos());

		if(dist < agentInfo.prefSpeed * timeStep)
		{
			agentInfo.prefVelocity.setSpeed(dist);
		} else
		{
			agentInfo.prefVelocity.setSpeed(agentInfo.prefSpeed);
		}

		if (abs(dist) > 1e-6)
		{
			agentInfo.prefVelocity.setSingle((currentGoal - agentInfo.GetPos()) / dist);
		} else
		{
			agentInfo.prefVelocity.setSpeed(0);
		}
		agentInfo.prefVelocity.setTarget(currentGoal);
	}

	DirectX::SimpleMath::Vector2 NavGraphComponent::GetClosestAvailablePoint(DirectX::SimpleMath::Vector2 p)
	{
		return p;
	}

	void NavGraphComponent::UpdateLocation(AgentSpatialInfo & agentInfo, AgentStruct& agentStruct, float deltaTime) const
	{
		Vector2 oldPos = agentInfo.GetPos() - agentInfo.GetVel() * deltaTime;

		float point_dist = Math::distanceToSegment(oldPos, agentInfo.GetPos(), agentStruct.route.points[agentStruct.pointsComplete]);
		if (abs(point_dist) < acceptanceRadius && agentStruct.pointsComplete < agentStruct.route.points.size() - 1) {
			agentStruct.pointsComplete++;
		}
	}

	std::shared_ptr<NavGraph> NavGraphComponent::GetNavGraph() const
	{
		return _navGraph;
	}

	size_t NavGraphComponent::getNodeId(size_t agentId) const
	{
		AgentSpatialInfo & agentInfo = _simulator->GetSpatialInfo(agentId);
		return _navGraph->GetClosestNodeIdByPosition(agentInfo.GetPos(), _navGraph->GetAllNodes());
	}
}