#include "Simulator.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"

#include "TacticComponent/NavMeshComponent.h"
#include "StrategyComponent/Goal/PointGoal.h"
#include "OperationComponent/HelbingComponent.h"

namespace FusionCrowd
{
	Simulator::Simulator()
	{
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

		navSystem.Update(timeStep);

		return true;
	}

	FusionCrowd::Agent * Simulator::getById(size_t id)
	{
		return &(agents.at(id)); // !!!!!!!!!!!!!!!!!!!!!!!
	}

	void Simulator::AddAgent(FusionCrowd::Agent agent)
	{
		agents.push_back(agent);
	}

	void Simulator::AddAgent(float maxAngleVel, float radius, float prefSpeed, float maxSpeed, float maxAccel,
	                         Vector2 pos)
	{
		FusionCrowd::Agent agent;
		agent.id = agents.size();
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
		agents.push_back(agent);
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
		std::vector<FusionCrowd::Agent *> agtPointers(agents.size());
		for (size_t a = 0; a < agents.size(); ++a)
		{
			agtPointers[a] = &agents[a];
		}

		//spatialQuerys[0]->SetAgents(agtPointers);
		//spatialQuerys[0]->ProcessObstacles();
	}

	Simulator::~Simulator()
	{
	}
}
