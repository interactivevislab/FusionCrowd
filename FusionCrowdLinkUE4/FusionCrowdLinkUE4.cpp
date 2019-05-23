#include "FusionCrowdLinkUE4.h"

#include "Math/consts.h"
#include "Math/Util.h"
#include "StrategyComponent/Goal/GoalSet.h"
#include "StrategyComponent/Goal/Goal.h"
#include "StrategyComponent/Goal/PointGoal.h"
#include "Simulator.h"
#include "Agent.h"
#include "OperationComponent/IOperationComponent.h"
#include "OperationComponent/HelbingComponent.h"
#include "OperationComponent/KaramouzasComponent.h"
#include "OperationComponent/ZanlungoComponent.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"
#include "TacticComponent/NavMeshComponent.h"

#include <algorithm>

using namespace DirectX::SimpleMath;

FusionCrowdLinkUE4::FusionCrowdLinkUE4(): agentsCount(0)
{
}

FusionCrowdLinkUE4::~FusionCrowdLinkUE4()
{
}

void FusionCrowdLinkUE4::StartFusionCrowd(char* navMeshDir)
{
	navMeshPath = (char*)malloc(strlen(navMeshDir) + 1);
	strcpy(navMeshPath, navMeshDir);

	sim = new FusionCrowd::Simulator();
	navMeshTactic = new FusionCrowd::NavMeshComponent(*sim, navMeshPath);
	kComponent = new FusionCrowd::Karamouzas::KaramouzasComponent(*sim);

	sim->AddOperComponent(*kComponent);
	sim->AddTacticComponent(*navMeshTactic);
}

int FusionCrowdLinkUE4::GetAgentCount()
{
	return sim->GetAgentCount();
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


	FusionCrowd::PointGoal* goal = new FusionCrowd::PointGoal(-3.0f, 5.0f);
	for(Vector2 pos : positions)
	{
		size_t id = sim->AddAgent(360, 0.19f, 0.05f, 0.2f, 5, pos, *goal);

		kComponent->AddAgent(id, 0.69f, 8.0f);
		navMeshTactic->AddAgent(id);
	}

	sim->InitSimulator();
}

void FusionCrowdLinkUE4::GetPositionAgents(agentInfo* agentsPos)
{
	FusionCrowd::NavSystem & nav = sim->GetNavSystem();
	bool info = sim->DoStep();
	agentsCount = sim->GetAgentCount();

	for (int i = 0; i < agentsCount; i++)
	{
		agentsPos[i].pos = new float[2];

		Vector2 buf = nav.GetSpatialInfo(i).pos;

		agentsPos[i].pos[0] = buf.x;
		agentsPos[i].pos[1] = buf.y;
	}
}