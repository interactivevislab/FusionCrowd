#include "Simulator.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"
#include "TacticComponent/NavMeshComponent.h"
#include "StrategyComponent/Goal/PointGoal.h"
#include "OperationComponent/HelbingComponent.h"

Simulator::Simulator()
{
}

bool Simulator::DoStep()
{
	Agents::PrefVelocity newVel;

	for (int i = 0; i < strategyComponents.size(); i++)
	{
		strategyComponents[i]->Update();
	}

	for (int i = 0; i < agents.size(); i++)
	{
		for (int j = 0; j < strategyComponents.size(); j++)
		{
			if (strategyComponents[j]->GetGoal(i) != NULL)
			{
				nav.SetPrefVelocity(&agents[i], strategyComponents[0]->GetGoal(i), newVel);
			}
		}

		agents[i]._velPref = newVel;

		ComputeNeighbors(&agents[i]);

		switch (agents[i]._operationComponent)
		{
		case 0:
		{
			operComponents[0]->Update(&agents[i], 0.1f);
			break;
		}
		case 1:
		{
			operComponents[1]->Update(&agents[i], 0.1f);
			break;
		}
		case 2:
		{
			operComponents[2]->Update(&agents[i], 0.1f);
			break;
		}
		case 3:
		{
			operComponents[3]->Update(&agents[i], 0.1f);
			break;
		}
		case 4:
		{
			operComponents[4]->Update(&agents[i], 0.1f);
			break;
		}
		case 5:
		{
			operComponents[5]->Update(&agents[i], 0.1f);
			break;
		}
		default:
			break;
		}

		nav._localizer->updateLocation(&agents[i]);
	}
	return true;
}

void Simulator::AddAgent(FusionCrowd::Agent agent)
{
	agents.push_back(agent);
}

void Simulator::AddAgent(float maxAngleVel, float maxNeighbors, int obstacleSet, float neighborDist, float radius,
	float prefSpeed, float maxSpeed, float maxAccel, Vector2 pos)
{
	FusionCrowd::Agent agent;
	agent._id = agents.size();
	agent._maxAngVel = maxAngleVel;
	agent._maxNeighbors = maxNeighbors;
	agent._obstacleSet = 1;
	agent._neighborDist = neighborDist;
	agent._radius = radius;
	agent._prefSpeed = prefSpeed;
	agent._maxSpeed = maxSpeed;
	agent._maxAccel = maxAccel;
	agent._pos = pos;
	agents.push_back(agent);
}

void Simulator::AddOperComponent(IOperationComponent* operComponent)
{
	operComponents.push_back(operComponent);
}

void Simulator::AddTacticComponent(ITacticComponent* tacticComponent)
{
	tacticComponents.push_back(tacticComponent);
}

void Simulator::AddSpatialQuery(FusionCrowd::SpatialQuery* spatialQuery)
{
	spatialQuerys.push_back(spatialQuery);
}

void Simulator::AddStrategyComponent(IStrategyComponent* strategyComponent)
{
	strategyComponents.push_back(strategyComponent);
}

void Simulator::AddNavComponent(std::string name, INavComponent* navComponent)
{
	navSystem.AddNavComponent(name, navComponent);
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

void Simulator::InitSimulator(const char* navMeshPath)
{
	nav._localizer = loadNavMeshLocalizer(navMeshPath, true);
	goal = new FusionCrowd::PointGoal(-3.0f, 5.0f);

	for (int i = 0; i < agents.size(); i++) {
		goal->setID(i);
		goal->assign(&agents[i]);
		nav._localizer->updateLocation(&agents[i]);
	}

	std::vector<FusionCrowd::Agent * > agtPointers(agents.size());
	for (size_t a = 0; a < agents.size(); ++a) {
		agtPointers[a] = &agents[a];
	}

	spatialQuerys[0]->SetAgents(agtPointers);
	spatialQuerys[0]->ProcessObstacles();
}

Simulator::~Simulator()
{
}
