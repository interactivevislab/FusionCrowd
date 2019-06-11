// TestFusionCrowd.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include "pch.h"

#include <iostream>
#include <random>
#include <memory>
#include <time.h>
#include <ctime>
#include <chrono>

#include "Agent.h"
#include "Simulator.h"
#include "Math/consts.h"

#include "StrategyComponent/Goal/PointGoal.h"

#include "OperationComponent/IOperationComponent.h"
#include "OperationComponent/KaramouzasComponent.h"
#include "OperationComponent/PedVOComponent.h"
#include "OperationComponent/ORCAComponent.h"

#include "Navigation/NavSystem.h"

using namespace DirectX::SimpleMath;

float RandFloat(float min, float max) {
	return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min)));
}

int main()
{
	std::string navPath = "Resources/square.nav";

	FusionCrowd::Simulator sim(navPath.c_str());
	auto kComponent = std::make_shared<FusionCrowd::Karamouzas::KaramouzasComponent>(sim);
	auto orcaComponent = std::make_shared<FusionCrowd::ORCA::ORCAComponent>(sim);
	auto pedvoComponent = std::make_shared<FusionCrowd::PedVO::PedVOComponent>(sim);

	sim.AddOperComponent(kComponent);
	sim.AddOperComponent(orcaComponent);
	sim.AddOperComponent(pedvoComponent);

	std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());
	//std::uniform_real_distribution<> uniform(-0.5f, 0.5f);

	//auto goal = std::make_shared<FusionCrowd::PointGoal>(-3.0f, 5.0f);

	//std::vector<Vector2> positions;
	//positions.push_back(Vector2(-0.55f, 4.0f));
	//positions.push_back(Vector2(-0.50f, -1.5f));
	//positions.push_back(Vector2(-0.1f, -1.5f));
	//positions.push_back(Vector2(-0.1f, -1.1f));
	//positions.push_back(Vector2(-0.5f, -1.1f));
	//positions.push_back(Vector2(0.3f, -1.1f));
	//positions.push_back(Vector2(0.3f, -1.5f));

	//std::srand (time(nullptr));
	//std::random_shuffle(positions.begin(), positions.end());

	//const int agentsCount = positions.size();
	//for(int i = 0; i < agentsCount; i++)
	//{
	//	//DirectX::SimpleMath::Vector2 pos(uniform(gen), uniform(gen));
	//	size_t id = sim.AddAgent(360, 0.19f, 0.05f, 0.2f, 5, positions[i], goal);

	//	//sim.SetOperationComponent(id, pedvoComponent->GetName());
	//	//if(i % 2 == 0)
	//	sim.SetOperationComponent(id, kComponent->GetName());
	//	//else
	//	//	sim.SetOperationComponent(id, orcaComponent->GetName());
	//}

	//---

	std::srand(time(nullptr));

	int input;
	std::cin >> input;

	int agentsInGroup = input / 4;
	int agentsCount = 4 * agentsInGroup;
	float agentsSpread = 100;
	float goalsDistance = 750;

	std::vector<Vector2> pointGoals;
	pointGoals.push_back(Vector2(0, goalsDistance));
	pointGoals.push_back(Vector2(goalsDistance, 0));
	pointGoals.push_back(Vector2(0, -goalsDistance));
	pointGoals.push_back(Vector2(-goalsDistance, 0));
	
	for (int goalIndex = 0; goalIndex < 4; goalIndex++) {
		std::vector<Vector2> positions;
		positions.reserve(agentsInGroup);
		for (int i = 0; i < agentsInGroup; i++) {
			Vector2 shift(RandFloat(-agentsSpread, agentsSpread), RandFloat(-agentsSpread, agentsSpread));
			positions.push_back(pointGoals[goalIndex] + shift);
		}

		auto goal = std::make_shared<FusionCrowd::PointGoal>(pointGoals[(goalIndex + 2) % 4].x, pointGoals[(goalIndex + 2) % 4].y);

		for(int i = 0; i < agentsInGroup; i++) {
			size_t id = sim.AddAgent(360, 0.19f, 0.05f, 0.2f, 5, positions[i], goal);
			sim.SetOperationComponent(id, kComponent->GetName());
		}
	}

	//---

	sim.InitSimulator();

	std::ofstream myfile;
    myfile.open ("traj.csv"); 
	const size_t stepsTotal = 1500;

	auto & navSystem = sim.GetNavSystem();

	using namespace std::chrono;
	long long measures[stepsTotal];

	for (int i = 0; i < stepsTotal; i++) {
		high_resolution_clock::time_point t1 = high_resolution_clock::now();
		if (!sim.DoStep()) break;
		high_resolution_clock::time_point t2 = high_resolution_clock::now();
		measures[i] = duration_cast<microseconds>(t2 - t1).count();
		/*for(size_t i = 0; i < agentsCount; i++)
		{
			auto agent = navSystem.GetPublicSpatialInfo(i);
			if(i > 0) myfile << ",";

			myfile << agent.posX << "," << agent.posY;
		}
		myfile << std::endl;*/
		if (i % 10 == 0) std::cout << i << std::endl;
	}

	std::sort(measures, measures + stepsTotal);
	std::cout << agentsCount
		<< " " << measures[0]
		<< " " << measures[stepsTotal / 4]
		<< " " << measures[stepsTotal / 2]
		<< " " << measures[stepsTotal * 3 / 4]
		<< " " << measures[stepsTotal - 1]
		<< std::endl;

	myfile.close();
}
