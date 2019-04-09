// TestFusionCrowd.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"

#include <iostream>

#include "Math/consts.h"
#include "Goal/GoalSet.h"
#include "Goal/Goal.h"
#include "Goal/PointGoal.h"
#include "Simulator.h"
#include "Agent.h"
#include "IOperComponent.h"
#include "OperationComponent/HelbingComponent.h"
#include "OperationComponent/KaramouzasComponent.h"
#include "OperationComponent/ZanlungoComponent.h"
#include "OperationComponent/PedVOComponent.h"
#include "OperationComponent/SpatialQuery/NavMeshSpatialQuery.h"
#include "NavComponents/NavMesh/NavMeshLocalizer.h"
#include "NavComponents/NavMeshCompnent.h"

using namespace DirectX::SimpleMath;

int main()
{
	std::string navPath = "Resources/simple.nav";
	FusionCrowd::Helbing::HelbingComponent* hComponent = new FusionCrowd::Helbing::HelbingComponent();
	FusionCrowd::PedVO::PedVOComponent* zComponent = new FusionCrowd::PedVO::PedVOComponent();
	FusionCrowd::NavMeshSpatialQuery* sq = new FusionCrowd::NavMeshSpatialQuery();

	NavMeshCompnent nav;
	nav._localizer = loadNavMeshLocalizer(navPath, true);
	sq->SetNavMeshLocalizer(nav._localizer);

	Simulator sim;
	sim.AddOperComponent(zComponent);
	sim.AddSpatialQuery(sq);

	const int agentsCount = 1;

	for(int i = 0; i < agentsCount; i++)
	{
		DirectX::SimpleMath::Vector2 pos(-0.5f + i * 0.25f, -1.0f  + i * 0.15f);
		sim.AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, pos);
	}

	for(int i = 0; i < agentsCount; i++)
		zComponent->AddAgent(i,3.0f, 0.1f, 1.0, true, 1.57f, 0.9f);

	sim.InitSimulator(navPath.c_str());

	while (sim.DoStep())
	{
		for(size_t i = 0; i < agentsCount; i++)
		{
			std::cout << i << " " << sim.agents[i]._pos.x << " " << sim.agents[i]._pos.y << std::endl;
		}
	}
}
