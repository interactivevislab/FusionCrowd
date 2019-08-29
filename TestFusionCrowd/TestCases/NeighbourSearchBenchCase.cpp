#include "pch.h"
#include "NeighbourSearchBenchCase.h"

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
#include "OperationComponent/ZanlungoComponent.h"

#include "Navigation/NavSystem.h"

#include "Benchmark/MicroscopicMetrics.h"


namespace TestFusionCrowd
{
	using namespace DirectX::SimpleMath;
	using namespace std::chrono;
	using namespace FusionCrowd;

	float NeighbourSearchBenchCase::RandFloat(float min, float max)
	{
		return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min)));
	}

	NeighbourSearchBenchCase::NeighbourSearchBenchCase()
	{
		for(size_t i = 0; i < 4; i++)
		{
			positions.push_back(std::vector<Vector2>());
		}
	}

	void NeighbourSearchBenchCase::Pre()
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

		std::cout << "totalAgents = " << totalAgents << std::endl;
	}

	void NeighbourSearchBenchCase::Run(const float & coeff)
	{
		std::string navPath = "Resources/square.nav";

		auto navSystem = std::make_shared<NavSystem>();
		navSystem->SetGridCoeff(coeff);
		navSystem->SetAgentsSensitivityRadius(searchRadius);

		auto kComponent = std::make_shared<FusionCrowd::Karamouzas::KaramouzasComponent>(navSystem);
		auto orcaComponent = std::make_shared<FusionCrowd::ORCA::ORCAComponent>(navSystem);
		auto pedvoComponent = std::make_shared<FusionCrowd::PedVO::PedVOComponent>(navSystem);

		FusionCrowd::Simulator & sim = FusionCrowd::Simulator{}
			.AddOpModel(kComponent)
			.AddOpModel(orcaComponent)
			.AddOpModel(pedvoComponent)
			.UseNavSystem(navSystem);

		for (int i = 0; i < totalAgents; i++) {
			auto goal = std::make_shared<FusionCrowd::PointGoal>(
				RandFloat(0, worldSide),
				RandFloat(0, worldSide)
			);
			sim.AddAgent(360, 0.19f, 0.05f, 0.2f, 5, Vector2(RandFloat(0, worldSide), RandFloat(0, worldSide)), goal);
			sim.SetOperationComponent(i, kComponent->GetName());
		}

		//sim.InitSimulator(navPath.c_str());

		std::ofstream myfile;
		myfile.open("way.csv");

		for (int i = 0; i < stepsTotal; i++)
		{
			high_resolution_clock::time_point t1 = high_resolution_clock::now();
			if (!sim.DoStep()) break;
			high_resolution_clock::time_point t2 = high_resolution_clock::now();
			measures[i] = duration_cast<microseconds>(t2 - t1).count();
		}

		myfile.close();

		recording = navSystem->GetRecording();
	}

	void NeighbourSearchBenchCase::Post()
	{
		std::ofstream myfile;
		myfile.open("measures.csv");
		for (int i = 0; i < stepsTotal; i++) {
			myfile << measures[i] << std::endl;
		}

		std::sort(measures, measures + stepsTotal);
		std::cout << agentsCount
			<< " " << measures[0]
			<< " " << measures[stepsTotal / 4]
			<< " " << measures[stepsTotal / 2]
			<< " " << measures[stepsTotal * 3 / 4]
			<< " " << measures[stepsTotal * 95 / 100]
			<< " " << measures[stepsTotal - 1]
			<< 1000000 / measures[stepsTotal / 2]
			<< std::endl;

		myfile.close();

		std::cout << "Latest recording, AvgDist: " << MicroscopicMetrics::AbsoluteDifference(*recording, *recording) << std::endl;
	}

	NeighbourSearchBenchCase::~NeighbourSearchBenchCase()
	{

	}
} // namespace TestFusionCrowd
