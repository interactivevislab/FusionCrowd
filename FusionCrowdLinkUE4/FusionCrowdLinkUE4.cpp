#include "FusionCrowdLinkUE4.h"

#include "Math/consts.h"
#include "Math/Util.h"

#include "OperationComponent/IOperationComponent.h"
#include "Export.h"

#include <algorithm>
#include <memory>

using namespace DirectX::SimpleMath;
using namespace FusionCrowd;

FusionCrowdLinkUE4::FusionCrowdLinkUE4()
{
}

FusionCrowdLinkUE4::~FusionCrowdLinkUE4()
{
	delete _strategy;
}

FusionCrowd::IStrategyComponent* ProxyStrategyFactory(FusionCrowd::ISimulatorFacade* simFacade, ComponentId assignedId, IStrategyComponent ** outStrategy)
{
	*outStrategy = new UE4StrategyProxy(simFacade, assignedId);
	return *outStrategy;
};

void FusionCrowdLinkUE4::StartFusionCrowd(char* navMeshDir)
{
	std::shared_ptr<ISimulatorBuilder> builder(BuildSimulator(), BuilderDeleter);
	builder->WithNavMesh(navMeshDir)
		->WithOp(ComponentIds::KARAMOUZAS_ID)
		->WithOp(ComponentIds::ORCA_ID)
		->WithOp(ComponentIds::PEDVO_ID)
		->WithOp(ComponentIds::HELBING_ID);

	builder->WithExternalStrategy(ProxyStrategyFactory, (IStrategyComponent **)(&_strategy));
	sim = std::shared_ptr<ISimulatorFacade>(builder->Build(), SimulatorFacadeDeleter);
}

int FusionCrowdLinkUE4::GetAgentCount()
{
	return sim->GetAgentCount();
}

size_t FusionCrowdLinkUE4::AddAgent(const float * agentPos, const float * goalPos, const char * opComponent)
{
	// TODO: use opComponent
	size_t id = sim->AddAgent(agentPos[0], agentPos[1], ComponentIds::KARAMOUZAS_ID, _strategy->GetId());
	sim->SetAgentGoal(id, goalPos[0], goalPos[1]);

	return id;
}

void FusionCrowdLinkUE4::SetGoal(size_t agentId, const float * goalPos)
{
	_strategy->SetGoal(agentId, goalPos);
}

void FusionCrowdLinkUE4::AddAgents(int agentsCount)
{
	std::vector<Vector2> positions;
	positions.push_back(Vector2(-0.55f, 4.0f));
	positions.push_back(Vector2(-0.50f, -1.5f));
	positions.push_back(Vector2(-0.1f, -1.5f));
	positions.push_back(Vector2(-0.1f, -1.1f));
	positions.push_back(Vector2(-0.5f, -1.1f));
	positions.push_back(Vector2(0.3f, -1.1f));
	positions.push_back(Vector2(0.3f, -1.5f));


	auto goal = Vector2(-3.0f, 5.0f);
	for(Vector2 pos : positions)
	{
		size_t id = sim->AddAgent(pos.x, pos.y, ComponentIds::PEDVO_ID, _strategy->GetId());

		sim->SetAgentGoal(id, goal.x, goal.y);
	}
}

void FusionCrowdLinkUE4::SetOperationModel(size_t agentId, const char * name)
{
	// TODO: implement this
}


void FusionCrowdLinkUE4::DoSimulationStep() {
	sim->DoStep();
}


void FusionCrowdLinkUE4::GetPositionAgents(agentInfo* ueAgentInfo)
{
	size_t agentsCount = sim->GetAgentCount();

	auto arr = sim->GetAgents();

	for (int i = 0; i < agentsCount; i++)
	{
		auto agent = arr[i];

		ueAgentInfo[i].id = agent.id;

		ueAgentInfo[i].pos = new float[2];
		ueAgentInfo[i].pos[0] = agent.posX;
		ueAgentInfo[i].pos[1] = agent.posY;

		ueAgentInfo[i].vel = new float[2];
		ueAgentInfo[i].vel[0] = agent.velX;
		ueAgentInfo[i].vel[1] = agent.velY;

		ueAgentInfo[i].orient = new float[2];
		ueAgentInfo[i].orient[0] = agent.orientX;
		ueAgentInfo[i].orient[1] = agent.orientY;

		ueAgentInfo[i].radius = agent.radius;

		// TODO: conversion
		auto name = std::to_string(agent.opCompId);

		ueAgentInfo[i].opCompName = new char [name.length() + 1];
		std::strcpy (ueAgentInfo[i].opCompName, name.c_str());
	}
}
