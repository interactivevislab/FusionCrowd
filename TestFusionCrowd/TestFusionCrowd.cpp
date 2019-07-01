// TestFusionCrowd.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include "pch.h"

#include <iostream>
#include <fstream>
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

#include "Benchmark/MicroscopicMetrics.h"

using namespace DirectX::SimpleMath;
using namespace std::chrono;
using namespace FusionCrowd;

const size_t stepsTotal = 100;
long long measures[stepsTotal];

const float worldSide = 100;
const int totalAgents = 5 * worldSide * worldSide;

const float goalsDistance = 10;
const int agentsInGroup = totalAgents / 2;
const int agentsCount = 2 * agentsInGroup;
const float agentsSpread = 1.f;
const float searchRadius = 5;

float RandFloat(float min, float max) {
	return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min)));
}

std::vector<std::vector<Vector2>> positions(4);
std::vector<Vector2> pointGoals;

void prep()
{
	pointGoals.push_back(Vector2(0, goalsDistance));
	pointGoals.push_back(Vector2(goalsDistance, 0));
	pointGoals.push_back(Vector2(0, -goalsDistance));
	pointGoals.push_back(Vector2(-goalsDistance, 0));

	for (int goalIndex = 0; goalIndex < 2; goalIndex++) {
		positions[goalIndex].reserve(agentsInGroup);
		for (int i = 0; i < agentsInGroup; i++) {
			float angle = (float)i / (float)agentsInGroup * 2 * 3.1415;
			Vector2 shift(agentsSpread * cos(angle), agentsSpread * sin(angle));
			positions[goalIndex].push_back(pointGoals[goalIndex] + shift);
		}
	}
}

void SwitchOperationComponent(FusionCrowd::Simulator & simulator, std::string componentName)
{
	for (int i = 0; i < agentsCount; i++) {
		simulator.SetOperationComponent(i, componentName);
	}
}

int control1 = 0;
int control2 = 0;

void AutoSelectOperationComponent(FusionCrowd::Simulator & simulator, int neighborsCount, std::string component1Name, std::string component2Name) {
	for (int i = 0; i < agentsCount; i++) {
		if (simulator.GetNavSystem().CountNeighbors(i) < neighborsCount) {
			simulator.SetOperationComponent(i, component1Name);
			control1++;
		} else {
			simulator.SetOperationComponent(i, component2Name);
			control2++;
		}
	}
}

void measure(float gridCellCoeff) 
{
	std::string navPath = "Resources/square.nav";

	FusionCrowd::Simulator sim(navPath.c_str());
	auto kComponent = std::make_shared<FusionCrowd::Karamouzas::KaramouzasComponent>(sim);
	auto orcaComponent = std::make_shared<FusionCrowd::ORCA::ORCAComponent>(sim);
	auto pedvoComponent = std::make_shared<FusionCrowd::PedVO::PedVOComponent>(sim);

	sim.AddOperComponent(kComponent);
	sim.AddOperComponent(orcaComponent);
	sim.AddOperComponent(pedvoComponent);

	//pointGoals.push_back(Vector2(0, goalsDistance));
	//pointGoals.push_back(Vector2(-goalsDistance, 0));
	//pointGoals.push_back(Vector2(0, -goalsDistance));
	//pointGoals.push_back(Vector2(goalsDistance, 0));
	//
	//for (int goalIndex = 0; goalIndex < 2; goalIndex++) 
	//{
	//	for(int i = 0; i < agentsInGroup; i++) {
	//		float angle = (float)i / (float)agentsInGroup * 2 * 3.1415;
	//		auto goal = std::make_shared<FusionCrowd::PointGoal>(
	//			pointGoals[(goalIndex + 2) % 4].x + agentsSpread * cos(angle),
	//			pointGoals[(goalIndex + 2) % 4].y + agentsSpread * sin(angle)
	//		);

	//		sim.AddAgent(360, 0.19f, 0.05f, 0.2f, 5, positions[goalIndex][i], goal);
	//	}
	//}

	for (int i = 0; i < totalAgents; i++) {
		auto goal = std::make_shared<FusionCrowd::PointGoal>(
			RandFloat(0, worldSide),
			RandFloat(0, worldSide)
		);
		sim.AddAgent(360, 0.19f, 0.05f, 0.2f, 5, Vector2(RandFloat(0, worldSide), RandFloat(0, worldSide)), goal);
	}

	SwitchOperationComponent(sim, kComponent->GetName());

	//---

	auto & navSystem = sim.GetNavSystem();
	navSystem.SetGridCoeff(gridCellCoeff);
	navSystem.SetAgentsSensitivityRadius(searchRadius);

	sim.InitSimulator();

	std::ofstream myfile;
	myfile.open("way.csv");

	for (int i = 0; i < stepsTotal; i++) {

		AutoSelectOperationComponent(sim, 4, kComponent->GetName(), orcaComponent->GetName());

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
		myfile << std::endl;

		std::cout << i << std::endl;*/
	}

	myfile.close();

	auto & rec = navSystem.GetRecording();
	std::cout << "AvgDist: " << MicroscopicMetrics::AbsoluteDifference(rec, rec) << std::endl;
}

void printResult()
{
	/*std::ofstream myfile;
	myfile.open("measures.csv");
	for (int i = 0; i < stepsTotal; i++) {
		myfile << measures[i] << std::endl;
	}
	myfile.close();*/

	std::sort(measures, measures + stepsTotal);
	std::cout
		<< " " << measures[0]
		<< " " << measures[stepsTotal / 4]
		<< " " << measures[stepsTotal / 2]
		<< " " << measures[stepsTotal * 3 / 4]
		<< " " << measures[stepsTotal * 95 / 100]
		<< " " << measures[stepsTotal - 1]
		/*<< "\tfps = "*/ << 1000000 / measures[stepsTotal / 2] << std::endl;

	//std::cout << control1 << ' ' << control2 << std::endl;
}

int main()
{
	prep();

	std::cout << "totalAgents = " << totalAgents << std::endl;
	for (float coeff = 0.5; coeff < 10; coeff += 0.25) {
		std::cout /*<< "coeff = "*/ << coeff << '\t';
		measure(coeff);
		printResult();
	}
}
