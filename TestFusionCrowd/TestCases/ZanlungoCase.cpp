#include "pch.h"
#include "ZanlungoCase.h"

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
#include "OperationComponent/ZanlungoComponent.h"

#include "Navigation/NavSystem.h"

#include "Benchmark/MicroscopicMetrics.h"

namespace TestFusionCrowd
{
	using namespace DirectX::SimpleMath;
	using namespace std::chrono;
	using namespace FusionCrowd;

	ZanlungoCase::ZanlungoCase()
	{
		for (size_t i = 0; i < 4; i++)
		{
			positions.push_back(std::vector<Vector2>());
		}
	}

	float ZanlungoCase::RandFloat(float min, float max)
	{
		return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min)));
	}

	void ZanlungoCase::Pre()
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

	void ZanlungoCase::Run(const float & coeff)
	{
		std::string navPath = "Resources/square.nav";

		FusionCrowd::Simulator sim(navPath.c_str());
		auto zanlungoComponent = std::make_shared<FusionCrowd::Zanlungo::ZanlungoComponent>(sim);

		sim.AddOperComponent(zanlungoComponent);

		for (int i = 0; i < (totalAgents / 2 - 1); i++) {
			auto goal = std::make_shared<FusionCrowd::PointGoal>(
				RandFloat(14.0f, 16.0f),
				RandFloat(10.0f, 20.0f)
				);
			sim.AddAgent(360, 0.19f, 0.05f, 0.2f, 5, Vector2(RandFloat(2.0f, 4.0f), RandFloat(10.0f, 20.0f)), goal);
		}

		for (int i = (totalAgents / 2 - 1); i < totalAgents; i++) {
			auto goal = std::make_shared<FusionCrowd::PointGoal>(
				RandFloat(8.0f, 10.0f),
				RandFloat(0.0f, 5.0f)
				);
			sim.AddAgent(360, 0.19f, 0.05f, 0.2f, 5, Vector2(RandFloat(8.0f, 10.0f), RandFloat(20.0f, 25.0f)), goal);
		}


		for (int i = 0; i < agentsCount; i++) {
			sim.SetOperationComponent(i, zanlungoComponent->GetName());
		}

		//---

		auto & navSystem = sim.GetNavSystem();
		navSystem.SetGridCoeff(coeff);
		navSystem.SetAgentsSensitivityRadius(searchRadius);

		sim.InitSimulator();

		std::ofstream myfile;
		myfile.open("way2.csv");

		for (int i = 0; i < stepsTotal; i++) {

			high_resolution_clock::time_point t1 = high_resolution_clock::now();
			if (!sim.DoStep()) break;
			high_resolution_clock::time_point t2 = high_resolution_clock::now();
			measures[i] = duration_cast<microseconds>(t2 - t1).count();

			for (size_t i = 0; i < agentsCount; i++)
			{
				auto agent = navSystem.GetPublicSpatialInfo(i);
				if (i > 0) myfile << ",";
				myfile << agent.posX << "," << agent.posY;
			}
			myfile << std::endl;
			std::cout << i << std::endl;
		}

		myfile.close();

		recording = navSystem.GetRecording();
	}

	void ZanlungoCase::Post()
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

		std::cout << control1 << ' ' << control2 << std::endl;

		myfile.close();

		std::cout << "Latest recording, AvgDist: " << MicroscopicMetrics::AbsoluteDifference(*recording, *recording) << std::endl;
	}

	ZanlungoCase::~ZanlungoCase()
	{
	}
}
