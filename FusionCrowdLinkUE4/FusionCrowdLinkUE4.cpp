#include "FusionCrowdLinkUE4.h"

#include "Math/consts.h"
#include "StrategyComponent/Goal/GoalSet.h"
#include "StrategyComponent/Goal/Goal.h"
#include "StrategyComponent/Goal/PointGoal.h"
#include "Simulator.h"
#include "Agent.h"
#include "OperationComponent/IOperationComponent.h"
#include "OperationComponent/HelbingComponent.h"
#include "OperationComponent/KaramouzasComponent.h"
#include "OperationComponent/ZanlungoComponent.h"
#include "Navigation/SpatialQuery/NavMeshSpatialQuery.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"
#include "TacticComponent/NavMeshComponent.h"

#include "Scenarios/DemoScenarios.h"

#include <algorithm>


FusionCrowdLinkUE4::FusionCrowdLinkUE4(): agentsCount(0)
{

}


FusionCrowdLinkUE4::~FusionCrowdLinkUE4()
{
}

void FusionCrowdLinkUE4::StartFusionCrowd(char* naVMeshDir)
{
	sim = new Simulator();
	navMeshPath = (char*)malloc(strlen(naVMeshDir) + 1);
	strcpy(navMeshPath, naVMeshDir);

	FusionCrowd::Zanlungo::ZanlungoComponent* zComponent = new FusionCrowd::Zanlungo::ZanlungoComponent();
	FusionCrowd::NavMeshSpatialQuery* sq = new FusionCrowd::NavMeshSpatialQuery();

	NavMeshComponent nav;
	nav._localizer = loadNavMeshLocalizer(naVMeshDir, true);
	sq->SetNavMeshLocalizer(nav._localizer);

	zComponent->AddAgent(0, 80.0f); //1
	zComponent->AddAgent(1, 80.0f); //1
	zComponent->AddAgent(2, 80.0f); //2
	zComponent->AddAgent(3, 80.0f); //3
	zComponent->AddAgent(4, 80.0f); //4
	zComponent->AddAgent(5, 80.0f); //5
	zComponent->AddAgent(6, 80.0f); //6

	sim->AddOperComponent(zComponent);
	sim->AddSpatialQuery(sq);
}

void FusionCrowdLinkUE4::StartSochiDemoScenario(char* naVMeshDir)
{
	sim = new Simulator();
	navMeshPath = (char*)malloc(strlen(naVMeshDir) + 1);
	strcpy(navMeshPath, naVMeshDir);
	agentsCount = 50;
	FusionCrowd::Scenarios::DemoScenarios::RunSochiDemoScenarioInitialize(sim, naVMeshDir, agentsCount);
}

int FusionCrowdLinkUE4::GetAgentCount()
{
	return sim->agents.size();
}

void FusionCrowdLinkUE4::AddAgent(int agentsCount)
{
	sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, DirectX::SimpleMath::Vector2(-0.55f, 4.0f));
	sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, DirectX::SimpleMath::Vector2(-0.50f, -1.5f));
	sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, DirectX::SimpleMath::Vector2(-0.1f, -1.5f));
	sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, DirectX::SimpleMath::Vector2(-0.1f, -1.1f));
	sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, DirectX::SimpleMath::Vector2(-0.5f, -1.1f));
	sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, DirectX::SimpleMath::Vector2(0.3f, -1.1f));
	sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, DirectX::SimpleMath::Vector2(0.3f, -1.5f));

	sim->InitSimulator(navMeshPath);
}

void FusionCrowdLinkUE4::GetPositionAgents(agentInfo* agentsPos)
{
	bool info = sim->DoStep();
	agentsCount = sim->agents.size();
	for (int i = 0; i < agentsCount; i++){
		DirectX::SimpleMath::Vector2 buf = sim->agents[i]._pos;
		agentsPos[i].pos[0] = buf.x;
		agentsPos[i].pos[1] = buf.y;
	}
}