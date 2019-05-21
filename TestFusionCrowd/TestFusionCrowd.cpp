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
	FusionCrowd::NavMeshComponent navMeshTactic(sim, navPath.c_str());
	FusionCrowd::Karamouzas::KaramouzasComponent kComponent(sim);

	sim.AddTacticComponent(navMeshTactic);
	sim.AddOperComponent(kComponent);

	const int agentsCount = 5;
	FusionCrowd::PointGoal goal(-3.0f, 5.0f);

	for(int i = 0; i < agentsCount; i++)
	{
		DirectX::SimpleMath::Vector2 pos(-0.5f + i * 0.25f, -1.0f  + i * 0.15f);
		size_t id = sim.AddAgent(360, 0.19f, 0.05f, 0.2f, 5, pos, goal);

		kComponent.AddAgent(id, 0.69f, 8.0f);
		navMeshTactic.AddAgent(id);
	}

	sim.InitSimulator();

	std::ofstream myfile;
    myfile.open ("traj.csv");
	size_t steps = 4000;
	while (sim.DoStep() && steps--)
	{
		for(size_t i = 0; i < agentsCount; i++)
		{
			if(i > 0) myfile << ",";

			myfile << sim.getById(i).pos.x << "," << sim.getById(i).pos.y;
		}
		myfile << std::endl;
	}

	myfile.close();
}
