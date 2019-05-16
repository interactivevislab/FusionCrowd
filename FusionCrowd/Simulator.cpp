#include "Simulator.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"

#include "TacticComponent/NavMeshComponent.h"
#include "StrategyComponent/Goal/PointGoal.h"
#include "OperationComponent/HelbingComponent.h"

namespace FusionCrowd
{
	Simulator::Simulator()
	{
		_navSystem = NavSystem();
	}

	bool Simulator::DoStep()
	{
		const float timeStep = 0.1f;
		for (IStrategyComponent & strategy : strategyComponents)
		{
			strategy.Update(timeStep);
		}

		for (ITacticComponent & tactic : tacticComponents)
		{
			tactic.Update(timeStep);
		}

		for (IOperationComponent & oper : operComponents)
		{
			oper.Update(timeStep);
		}

		_navSystem.Update(timeStep);

		return true;
	}

	Agent & Simulator::getById(size_t id)
	{
		return _agents.at(id);
	}

	NavSystem & Simulator::GetNavSystem()
	{
		return _navSystem;
	}

	void Simulator::AddAgent(float maxAngleVel, float radius, float prefSpeed, float maxSpeed, float maxAccel, Vector2 pos, Goal & g)
	{
		Agent agent(g);
		agent.id = _agents.size();
		agent.maxAngVel = maxAngleVel;

		/*
		agent._maxNeighbors = maxNeighbors;
		agent._obstacleSet = 1;
		agent._neighborDist = neighborDist;

		*/
		agent.radius = radius;
		agent.prefSpeed = prefSpeed;
		agent.maxSpeed = maxSpeed;
		agent.maxAccel = maxAccel;
		agent.pos = pos;
		_agents.push_back(agent);
	}

	void Simulator::AddOperComponent(IOperationComponent & component)
	{
		operComponents.push_back(component);
	}

	void Simulator::AddTacticComponent(ITacticComponent & component)
	{
		tacticComponents.push_back(component);
	}

	void Simulator::AddStrategyComponent(IStrategyComponent& component)
	{
		strategyComponents.push_back(component);
	}

	/*
	void Simulator::AddSpatialQuery(FusionCrowd::SpatialQuery* spatialQuery)
	{
		spatialQuerys.push_back(spatialQuery);
	}

	void Simulator::ComputeNeighbors(FusionCrowd::Agent * agent)
	{
		agent->StartQuery();
		spatialQuerys[0]->ObstacleQuery(agent);
		// agents
		if (agent->_maxNeighbors > 0) {
			spatialQuerys[0]->AgentQuery(agent);
		}
	}
	*/

	void Simulator::InitSimulator()
	{
		std::vector<FusionCrowd::Agent *> agtPointers(_agents.size());
		for (size_t a = 0; a < _agents.size(); ++a)
		{
			agtPointers[a] = &_agents[a];
		}

		//spatialQuerys[0]->SetAgents(agtPointers);
		//spatialQuerys[0]->ProcessObstacles();
	}

	Simulator::~Simulator()
	{
	}
}
