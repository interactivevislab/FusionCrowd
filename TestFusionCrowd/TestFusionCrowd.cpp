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
#include "OperationComponent/SpatialQuery/NavMeshSpatialQuery.h"
#include "NavComponents/NavMesh/NavMeshLocalizer.h"
#include "NavComponents/NavMeshCompnent.h"

int main()
{
	FusionCrowd::Helbing::HelbingComponent* hComponent = new FusionCrowd::Helbing::HelbingComponent();
	FusionCrowd::Karamouzas::KaramouzasComponent* kComponent = new FusionCrowd::Karamouzas::KaramouzasComponent();
	FusionCrowd::NavMeshSpatialQuery* sq = new FusionCrowd::NavMeshSpatialQuery();

	NavMeshCompnent nav;
	nav._localizer = loadNavMeshLocalizer("D:/Lebin/Menge-master/examples/core/navMesh/simple.nav", true);
	sq->SetNavMeshLocalizer(nav._localizer);

	Simulator sim;
	//sim.AddOperComponent(hComponent);
	sim.AddOperComponent(kComponent);
	sim.AddSpatialQuery(sq);
	sim.AddAgent(360, 10, 1, 5, 0.19f, 1.04f, 2, 5, FusionCrowd::Math::Vector2(-0.55f, 4.0f)); //0
	//sim.AddAgent(360, 10, 1, 5, 0.19f, 1.04f, 2, 5, FusionCrowd::Math::Vector2(-0.50f, -1.5f));//1
	//sim.AddAgent(360, 10, 1, 5, 0.19f, 1.04f, 2, 5, FusionCrowd::Math::Vector2(-0.1f, -1.5f)); //2
	//sim.AddAgent(360, 10, 1, 5, 0.19f, 1.04f, 2, 5, FusionCrowd::Math::Vector2(-0.1f, -1.1f)); //3
	//sim.AddAgent(360, 10, 1, 5, 0.19f, 1.04f, 2, 5, FusionCrowd::Math::Vector2(-0.5f, -1.1f)); //4
	//sim.AddAgent(360, 10, 1, 5, 0.19f, 1.04f, 2, 5, FusionCrowd::Math::Vector2(0.3f, -1.1f)); //5
	//sim.AddAgent(360, 10, 1, 5, 0.19f, 1.04f, 2, 5, FusionCrowd::Math::Vector2(0.3f, -1.5f)); //6

	kComponent->AddAgent(0, 0.69f, 8.0f); //0
	//kComponent->AddAgent(1, 0.69f, 8.0f); //1
	//kComponent->AddAgent(2, 0.69f, 8.0f); //2
	//kComponent->AddAgent(3, 0.69f, 8.0f); //3
	//kComponent->AddAgent(4, 0.69f, 8.0f); //4
	//kComponent->AddAgent(5, 0.69f, 8.0f); //5
	//kComponent->AddAgent(6, 0.69f, 8.0f); //6

	sim.InitSimulator();


	bool running = true;
	while (running) {
		running = sim.DoStep();
		std::cout << "0 " << sim.agents[0]._pos._x << " " << sim.agents[0]._pos._y << "\n";
		//std::cout << "1 " << sim.agents[1]._pos._x << " " << sim.agents[1]._pos._y << "\n";
		//std::cout << "2 " << sim.agents[2]._pos._x << " " << sim.agents[2]._pos._y << "\n";
		//std::cout << "3 " << sim.agents[3]._pos._x << " " << sim.agents[3]._pos._y << "\n";
		//std::cout << "4 " << sim.agents[4]._pos._x << " " << sim.agents[4]._pos._y << "\n";
		//std::cout << "5 " << sim.agents[5]._pos._x << " " << sim.agents[5]._pos._y << "\n";
		//std::cout << "6 " << sim.agents[6]._pos._x << " " << sim.agents[6]._pos._y << "\n";
	}
 //   std::cout << "Hello World!\n";
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started:
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
