#include "FusionCrowdLinkUE4.h"

#include "Math/consts.h"
#include "Goal/GoalSet.h"
#include "Goal/Goal.h"
#include "Goal/PointGoal.h"
#include "Simulator.h"
#include "Agent.h"
#include "IOperComponent.h"
#include "OperationComponent/HelbingComponent.h"
#include "OperationComponent/SpatialQuery/NavMeshSpatialQuery.h"
#include "NavComponents/NavMesh/NavMeshLocalizer.h"
#include "NavComponents/NavMeshCompnent.h"


FusionCrowdLinkUE4::FusionCrowdLinkUE4(): agentsCount(0)
{
}


FusionCrowdLinkUE4::~FusionCrowdLinkUE4()
{
}

void FusionCrowdLinkUE4::StartFusionCrowd(char* naVMeshDir)
{
	sim = new Simulator();
	FusionCrowd::Helbing::HelbingComponent* hComponent = new FusionCrowd::Helbing::HelbingComponent();
	FusionCrowd::NavMeshSpatialQuery* sq = new FusionCrowd::NavMeshSpatialQuery();

	NavMeshCompnent nav;
	nav._localizer = loadNavMeshLocalizer(naVMeshDir, true);
	sq->SetNavMeshLocalizer(nav._localizer);

	IOperComponent* tes = hComponent;

	sim->AddOperComponent(hComponent);
	sim->AddSpatialQuery(sq);
}

int FusionCrowdLinkUE4::GetAgentCount()
{
	return sim->agents.size();
}

void FusionCrowdLinkUE4::AddAgent(int agentsCount)
{
	sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, FusionCrowd::Math::Vector2(-0.55f, 4.0f));
	sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, FusionCrowd::Math::Vector2(-0.50f, -1.5f));
	sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, FusionCrowd::Math::Vector2(-0.1f, -1.5f));
	sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, FusionCrowd::Math::Vector2(-0.1f, -1.1f));
	sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, FusionCrowd::Math::Vector2(-0.5f, -1.1f));
	sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, FusionCrowd::Math::Vector2(0.3f, -1.1f));
	sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, FusionCrowd::Math::Vector2(0.3f, -1.5f));
	sim->InitSimulator();
}

void FusionCrowdLinkUE4::GetPositionAgents(agentInfo* agentsPos)
{
	bool info = sim->DoStep();
	agentsCount = sim->agents.size();
	for (int i = 0; i < agentsCount; i++){
		agentsPos[i].pos = new float[2];
		FusionCrowd::Math::Vector2 buf = sim->agents[i]._pos;
		agentsPos[i].pos[0] = buf.x();
		agentsPos[i].pos[1] = buf.y();
	}
}