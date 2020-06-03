#include "NavGraphComponent.h"
#include "Navigation/AgentSpatialInfo.h"


using namespace DirectX::SimpleMath;

namespace FusionCrowd
{

	NavGraphComponent::NavGraphComponent(std::shared_ptr<Simulator> simulator,
		std::shared_ptr<NavSystem> navSystem) : _simulator(simulator), _navSystem(navSystem), _navGraph(navSystem->GetNavGraph())
	{ }

	void NavGraphComponent::AddAgent(size_t id)
	{
		size_t curNodeId = getNodeId(id);
		NavGraphPathPlanner pathPlanner(_navGraph);
		AgentStruct agtStruct;
		agtStruct.id = id;
		agtStruct.route = pathPlanner.GetRoute(curNodeId, curNodeId);
		agtStruct.nodesComplete = 0;
		agtStruct.goalNodeID = agtStruct.route->nodes.at(agtStruct.nodesComplete);
		_agents.push_back(agtStruct);
	}

	void NavGraphComponent::Replan(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct)
	{
		auto & agentGoal = _simulator->GetAgentGoal(agentStruct.id);
		size_t curNodeId = _navGraph->GetClosestNodeIdByPosition(agentInfo.pos, _navGraph->GetAllNodes());
		size_t goalNodeId = _navGraph->GetClosestNodeIdByPosition(agentGoal.getCentroid(), _navGraph->GetAllNodes());
		NavGraphPathPlanner pathPlanner(_navGraph);
		agentStruct.route = pathPlanner.GetRoute(curNodeId, goalNodeId);
		agentStruct.nodesComplete = 0;
		agentStruct.goalNodeID = agentStruct.route->nodes.at(agentStruct.route->nodes.size() - agentStruct.nodesComplete - 1);
	}

	bool NavGraphComponent::DeleteAgent(size_t id)
	{
		return false;
	}

	void NavGraphComponent::Update(float timeStep)
	{
		for (auto & agtStruct : _agents)
		{
			size_t id = agtStruct.id;
			AgentSpatialInfo & info = _simulator->GetSpatialInfo(id);

			if (agtStruct.route->nodes.size() < 2 )
			{
				Replan(info, agtStruct);
			}

			SetPrefVelocity(info, agtStruct, timeStep);
			UpdateLocation(info, agtStruct, timeStep);
		}
	}

	void NavGraphComponent::SetPrefVelocity(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct, float timeStep)
	{

		NavGraphNode goalNode = _navGraph->GetNode(agentStruct.goalNodeID);
		
		Vector2 shift = { -1*agentInfo.orient.y, agentInfo.orient.x };
		shift.Normalize();
		shift *= agentInfo.radius * 4;
		float dist = goalNode.position.Distance(goalNode.position + shift, agentInfo.pos);
		
		if(dist < agentInfo.prefSpeed * timeStep)
		{
			agentInfo.prefVelocity.setSpeed(dist);
		} else
		{
			agentInfo.prefVelocity.setSpeed(agentInfo.prefSpeed);
		}

		if (abs(dist) > 1e-6)
		{
			agentInfo.prefVelocity.setSingle((goalNode.position + shift - agentInfo.pos) / dist);
		} else
		{
			agentInfo.prefVelocity.setSpeed(1e-6);
		}

		TrafficLightsBunch* curLights = _navSystem->GetTrafficLights(agentStruct.goalNodeID);
		if (curLights)
		{
			if ((curLights->GetProperLight(agentInfo.orient)->GetCurLight() == TrafficLight::red || 
				curLights->GetProperLight(agentInfo.orient)->GetCurLight() == TrafficLight::yellow) && 
				dist < agentInfo.radius*15 && dist > agentInfo.radius * 12)
			{
				agentInfo.prefVelocity.setSpeed(1e-6);
			}
			
		}
	}

	DirectX::SimpleMath::Vector2 NavGraphComponent::GetClosestAvailablePoint(DirectX::SimpleMath::Vector2 p)
	{
		return p;
	}

	void NavGraphComponent::UpdateLocation(AgentSpatialInfo & agentInfo, AgentStruct& agentStruct, float deltaTime) const
	{
		NavGraphNode goalNode = _navGraph->GetNode(agentStruct.goalNodeID);
		float dist = goalNode.position.Distance(goalNode.position, agentInfo.pos);
		if (abs(dist) < 1.0 && agentStruct.nodesComplete < agentStruct.route->nodes.size()-1)
		{
			agentStruct.nodesComplete++;

			agentStruct.goalNodeID = agentStruct.route->nodes.at(agentStruct.route->nodes.size() - agentStruct.nodesComplete - 1);
		}

	}

	std::shared_ptr<NavGraph> NavGraphComponent::GetNavGraph() const
	{
		return _navGraph;
	}

	unsigned int NavGraphComponent::getNodeId(size_t agentId) const
	{
		AgentSpatialInfo & agentInfo = _simulator->GetSpatialInfo(agentId);
		return _navGraph->GetClosestNodeIdByPosition(agentInfo.pos, _navGraph->GetAllNodes());
	}

	unsigned int NavGraphComponent::getGoalNodeId(size_t agentId) const
	{
		for (auto agtStruct : _agents)
		{
			if (agentId == agtStruct.id)
			{
				return agtStruct.goalNodeID;
			}
		}

		return 0;
	}
}