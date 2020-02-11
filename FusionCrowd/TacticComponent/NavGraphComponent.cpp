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
			//auto & agentGoal = _simulator->GetAgentGoal(id);
			AgentSpatialInfo& agentInfo = _simulator->GetSpatialInfo(id);
			size_t curNodeId = _navGraph->GetClosestNodeIdByPosition(agentInfo.pos, _navGraph->GetAllNodes());
			//size_t goalNodeId = _navGraph->GetClosestNodeIdByPosition(agentGoal.getCentroid(), _navGraph->GetAllNodes());
			NavGraphPathPlanner pathPlanner(_navGraph);
			AgentStruct agtStruct;
			agtStruct.id = id;
			//agtStruct.routeNodes = &pathPlanner.GetRoute(curNodeId, curNodeId)->nodes;
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

			SetPrefVelocity(info, agtStruct);
			UpdateLocation(info, agtStruct, timeStep);
		}
	}

	void NavGraphComponent::SetPrefVelocity(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct)
	{
		agentInfo.prefVelocity.setSpeed(agentInfo.prefSpeed);
		NavGraphNode goalNode = _navGraph->GetNode(agentStruct.goalNodeID);
		float dist = goalNode.position.Distance(goalNode.position, agentInfo.pos);
		Vector2 dir = { 1,1 };
		if (abs(dist) > 1.0e-6)
		{
			agentInfo.prefVelocity.setSingle((goalNode.position - agentInfo.pos) / dist);
			dir = (goalNode.position - agentInfo.pos) / dist;
		}

		agentInfo.vel = 3 * dir; // hardcode velocity, need to be changed
		agentInfo.velNew = 3 * dir;
		
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



}