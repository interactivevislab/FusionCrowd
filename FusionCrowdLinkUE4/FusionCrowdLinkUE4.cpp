#include "FusionCrowdLinkUE4.h"

#include "Simulator.h"

#include "Math/consts.h"
#include "Math/Util.h"

#include "StrategyComponent/Goal/GoalSet.h"
#include "StrategyComponent/Goal/Goal.h"
#include "StrategyComponent/Goal/PointGoal.h"

#include "OperationComponent/IOperationComponent.h"
#include "OperationComponent/KaramouzasComponent.h"
#include "OperationComponent/ORCAComponent.h"
#include "OperationComponent/PedVOComponent.h"
#include "OperationComponent/HelbingComponent.h"

#include "TacticComponent/NavMeshComponent.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"
#include "Navigation/NavSystem.h"

#include <algorithm>
#include <memory>

using namespace DirectX::SimpleMath;
using namespace FusionCrowd;

FusionCrowdLinkUE4::FusionCrowdLinkUE4(): agentsCount(0)
{
}

FusionCrowdLinkUE4::~FusionCrowdLinkUE4()
{
}

void FusionCrowdLinkUE4::StartFusionCrowd(char* navMeshDir)
{

	auto path = std::string(navMeshDir);

	auto localizer = std::make_shared<NavMeshLocalizer>(path, true);
	auto navSystem = std::make_shared<NavSystem>(localizer);
	navSystem->Init();

	sim = std::make_shared<Simulator>();
	sim->UseNavSystem(navSystem);

	kComponent = std::make_shared<FusionCrowd::Karamouzas::KaramouzasComponent>(navSystem);
	orcaComponent = std::make_shared<FusionCrowd::ORCA::ORCAComponent>(navSystem);
	pedvoComponent = std::make_shared<FusionCrowd::PedVO::PedVOComponent>(navSystem);
	helbingComponent = std::make_shared<FusionCrowd::Helbing::HelbingComponent>(navSystem);

	sim->AddOpModel(kComponent);
	sim->AddOpModel(orcaComponent);
	sim->AddOpModel(pedvoComponent);
	sim->AddOpModel(helbingComponent);

	_tactic = std::make_shared<FusionCrowd::NavMeshComponent>(sim, localizer);
	sim->AddTactic(_tactic);

	_strategy = std::make_shared<UE4StrategyProxy>(sim);
	sim->AddStrategy(_strategy);
}

int FusionCrowdLinkUE4::GetAgentCount()
{
	return sim->GetAgentCount();
}

size_t FusionCrowdLinkUE4::AddAgent(const float * agentPos, const float * goalPos, const char * opComponent)
{
	Vector2 position(agentPos[0], agentPos[1]);
	auto goal = std::make_shared<FusionCrowd::PointGoal>(goalPos[0], goalPos[1]);
	std::string compName(opComponent);

	size_t id = sim->AddAgent(360, 0.19f, 0.3f, 0.5f, 5, position, goal);

	sim->SetOperationComponent(id, compName);
	sim->SetTacticComponent(id, _tactic->GetName());
	sim->SetStrategyComponent(id, _strategy->GetName());

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


	auto goal = std::make_shared<FusionCrowd::PointGoal>(-3.0f, 5.0f);
	for(Vector2 pos : positions)
	{
		size_t id = sim->AddAgent(360, 0.19f, 0.05f, 0.2f, 5, pos, goal);

		sim->SetOperationComponent(id, pedvoComponent->GetName());
		sim->SetTacticComponent(id, _tactic->GetName());
		sim->SetStrategyComponent(id, _strategy->GetName());
	}
}

void FusionCrowdLinkUE4::SetOperationModel(size_t agentId, const char * name)
{
	sim->SetOperationComponent(agentId, std::string(name));
}


void FusionCrowdLinkUE4::GetPositionAgents(agentInfo* ueAgentInfo)
{
	std::shared_ptr<NavSystem> nav = sim->GetNavSystem();
	bool info = sim->DoStep();
	agentsCount = sim->GetAgentCount();

	for (int i = 0; i < agentsCount; i++)
	{
		auto spatialInfo = nav->GetPublicSpatialInfo(i);

		ueAgentInfo[i].id = spatialInfo.id;

		ueAgentInfo[i].pos = new float[2];
		ueAgentInfo[i].pos[0] = spatialInfo.posX;
		ueAgentInfo[i].pos[1] = spatialInfo.posY;

		ueAgentInfo[i].vel = new float[2];
		ueAgentInfo[i].vel[0] = spatialInfo.velX;
		ueAgentInfo[i].vel[1] = spatialInfo.velY;

		ueAgentInfo[i].orient = new float[2];
		ueAgentInfo[i].orient[0] = spatialInfo.orientX;
		ueAgentInfo[i].orient[1] = spatialInfo.orientY;

		ueAgentInfo[i].radius = spatialInfo.radius;

		auto opComp = sim->GetAgent(i).opComponent;
		auto name = opComp != nullptr ? opComp->GetName() : std::string("NO_OP_COMPONENT");

		ueAgentInfo[i].opCompName = new char [name.length() + 1];
		std::strcpy (ueAgentInfo[i].opCompName, name.c_str());
	}
}
