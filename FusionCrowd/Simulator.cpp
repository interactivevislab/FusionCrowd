#include "Simulator.h"
#include "NavComponents/NavMeshCompnent.h"
#include "Goal/PointGoal.h"
#include "NavComponents/NavMesh/NavMeshLocalizer.h"
#include "NavComponents/NavMeshCompnent.h"
#include "OperationComponent/HelbingComponent.h"

Simulator::Simulator()
{
}

bool Simulator::DoStep()
{
	Agents::PrefVelocity newVel;

	for (int i = 0; i < agents.size(); i++)
	{
		nav.SetPrefVelocity(&agents[i], goal, newVel);
		agents[i]._velPref = newVel;

		ComputeNeighbors(&agents[i]);
		operComponents[0]->Update(&agents[i], 0.1f);
		nav._localizer->updateLocation(&agents[i]);
	}
	return true;
}

void Simulator::AddAgent(FusionCrowd::Agent agent)
{
	agents.push_back(agent);
}

void Simulator::AddAgent(float maxAngleVel, float maxNeighbors, int obstacleSet, float neighborDist, float radius,
	float prefSpeed, float maxSpeed, float maxAccel, FusionCrowd::Math::Vector2 pos)
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

void Simulator::AddOperComponent(IOperComponent* operComponent)
{
	operComponents.push_back(operComponent);
}

void Simulator::AddSpatialQuery(FusionCrowd::SpatialQuery* spatialQuery)
{
	spatialQuerys.push_back(spatialQuery);
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

void Simulator::InitSimulator()
{
	nav._localizer = loadNavMeshLocalizer("D:/Lebin/Menge-master/examples/core/navMesh/simple.nav", true);
	goal = new FusionCrowd::PointGoal(0.5f, -0.5f);

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
