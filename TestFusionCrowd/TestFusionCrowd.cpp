// TestFusionCrowd.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"

#include <iostream>

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

using namespace DirectX::SimpleMath;

int main()
{
	std::string navPath = "Resources/simple.nav";
	FusionCrowd::Helbing::HelbingComponent* hComponent = new FusionCrowd::Helbing::HelbingComponent();
	FusionCrowd::Zanlungo::ZanlungoComponent* zComponent = new FusionCrowd::Zanlungo::ZanlungoComponent();
	FusionCrowd::NavMeshSpatialQuery* sq = new FusionCrowd::NavMeshSpatialQuery();

	NavMeshComponent nav;
	nav._localizer = loadNavMeshLocalizer(navPath, true);
	sq->SetNavMeshLocalizer(nav._localizer);

	Simulator sim;
	sim.AddOperComponent(zComponent);
	sim.AddSpatialQuery(sq);

	const int agentsCount = 6;

	for(int i = 0; i < agentsCount; i++)
	{
		DirectX::SimpleMath::Vector2 pos(-0.5f + i * 0.25f, -1.0f  + i * 0.15f);
		sim.AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, pos);
	}

	for(int i = 0; i < agentsCount; i++)
		zComponent->AddAgent(i, 80.0f);

	sim.InitSimulator(navPath.c_str());

	while (sim.DoStep())
	{
		for(size_t i = 0; i < agentsCount; i++)
		{
			std::cout << i << " " << sim.agents[i]._pos.x << " " << sim.agents[i]._pos.y << std::endl;
		}
	}
}
