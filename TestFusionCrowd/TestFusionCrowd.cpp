// TestFusionCrowd.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"

#include <iostream>

#include "Agent.h"
#include "Simulator.h"
#include "Math/consts.h"
#include "StrategyComponent/Goal/GoalSet.h"
#include "StrategyComponent/Goal/Goal.h"
#include "StrategyComponent/Goal/PointGoal.h"
#include "TacticComponent/NavMeshComponent.h"
#include "OperationComponent/IOperationComponent.h"
#include "OperationComponent/HelbingComponent.h"
#include "OperationComponent/KaramouzasComponent.h"
#include "OperationComponent/ZanlungoComponent.h"
#include "OperationComponent/PedVOComponent.h"
#include "Navigation/SpatialQuery/NavMeshSpatialQuery.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"

using namespace DirectX::SimpleMath;

int main()
{
	std::string navPath = "Resources/simple.nav";

	FusionCrowd::Simulator sim;
	FusionCrowd::NavSystem nav;
	FusionCrowd::NavMeshComponent navMeshTactic(sim, navPath.c_str());
	FusionCrowd::Karamouzas::KaramouzasComponent kComponent(nav);

	sim.AddOperComponent(kComponent);

	const int agentsCount = 10;

	for(int i = 0; i < agentsCount; i++)
	{
		DirectX::SimpleMath::Vector2 pos(-0.5f + i * 0.25f, -1.0f  + i * 0.15f);
		sim.AddAgent(360, 10, 1, 5, 0.19f, pos);
	}

	for(int i = 0; i < agentsCount; i++)
		kComponent.AddAgent(i, 0.1f, 0.1f);

	sim.InitSimulator();

	while (sim.DoStep())
	{
		for(size_t i = 0; i < agentsCount; i++)
		{
			std::cout << i << " " << sim.agents[i].pos.x << " " << sim.agents[i].pos.y << std::endl;
		}
	}
}
