#include "FusionCrowdLinkUE4.h"


FusionCrowdLinkUE4::FusionCrowdLinkUE4(): agentsCount(0)
{
}


FusionCrowdLinkUE4::~FusionCrowdLinkUE4()
{
}

void FusionCrowdLinkUE4::StartFusionCrowd()
{
	FusionCrowd::Helbing::HelbingComponent* hComponent = new FusionCrowd::Helbing::HelbingComponent();
	FusionCrowd::NavMeshSpatialQuery* sq = new FusionCrowd::NavMeshSpatialQuery();

	NavMeshCompnent nav;
	nav._localizer = loadNavMeshLocalizer("D:/Lebin/Menge-master/examples/core/navMesh/simple.nav", true);
	sq->SetNavMeshLocalizer(nav._localizer);

	IOperComponent* tes = hComponent;

	sim.AddOperComponent(hComponent);
	sim.AddSpatialQuery(sq);
}

void FusionCrowdLinkUE4::DoStep()
{

}

int FusionCrowdLinkUE4::GetAgentCount()
{
	return sim.agents.size();
}

void FusionCrowdLinkUE4::AddAgent(int agentsCount)
{
	sim.AddAgent(360, 10, 1, 5, 0.19f, 1.04f, 2, 5, FusionCrowd::Math::Vector2(-0.55f, 4.0f));
	sim.AddAgent(360, 10, 1, 5, 0.19f, 1.04f, 2, 5, FusionCrowd::Math::Vector2(-0.50f, -1.5f));
	sim.AddAgent(360, 10, 1, 5, 0.19f, 1.04f, 2, 5, FusionCrowd::Math::Vector2(-0.1f, -1.5f));
	sim.AddAgent(360, 10, 1, 5, 0.19f, 1.04f, 2, 5, FusionCrowd::Math::Vector2(-0.1f, -1.1f));
	sim.AddAgent(360, 10, 1, 5, 0.19f, 1.04f, 2, 5, FusionCrowd::Math::Vector2(-0.5f, -1.1f));
	sim.AddAgent(360, 10, 1, 5, 0.19f, 1.04f, 2, 5, FusionCrowd::Math::Vector2(0.3f, -1.1f));
	sim.AddAgent(360, 10, 1, 5, 0.19f, 1.04f, 2, 5, FusionCrowd::Math::Vector2(0.3f, -1.5f));
	sim.InitSimulator();
}

void FusionCrowdLinkUE4::GetPositionAgents(agentInfo* agentsPos)
{
	sim.DoStep();

	for (int i = 0; i < agentsCount; i++){
		agentsPos[i].pos = new float[2];
		agentsPos[i].pos[0] = sim.agents[i]._pos._x;
		agentsPos[i].pos[1] = sim.agents[i]._pos._y;
	}
}