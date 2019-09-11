#include "pch.h"
#include "ZanlungoCase.h"

#include <iostream>
#include <fstream>
#include <random>
#include <memory>
#include <time.h>
#include <ctime>
#include <chrono>

#include "Math/consts.h"
#include "Benchmark/MicroscopicMetrics.h"
#include "Export.h"

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

		std::shared_ptr<ISimulatorBuilder> builder(BuildSimulator(), BuilderDeleter);
		builder->WithNavMesh("Resources/square.nav")
			->WithOp(FusionCrowd::ZANLUNGO_ID);

		std::shared_ptr<ISimulatorFacade> sim(builder->Build(), SimulatorFacadeDeleter);

		for (int i = 0; i < (totalAgents / 2 - 1); i++)
		{
			size_t id = sim->AddAgent(RandFloat(2.0f, 4.0f), RandFloat(10.0f, 20.0f), ZANLUNGO_ID, -1);
			sim->SetAgentGoal(id, RandFloat(14.0f, 16.0f), RandFloat(10.0f, 20.0f));
		}

		for (int i = (totalAgents / 2 - 1); i < totalAgents; i++)
		{
			size_t id = sim->AddAgent(RandFloat(8.0f, 10.0f), RandFloat(20.0f, 25.0f), ZANLUNGO_ID, -1);
			sim->SetAgentGoal(id, RandFloat(8.0f, 10.0f), RandFloat(0.0f, 5.0f));
		}

		std::ofstream myfile;
		myfile.open("way2.csv");

		for (int i = 0; i < stepsTotal; i++)
		{
			high_resolution_clock::time_point t1 = high_resolution_clock::now();
			sim->DoStep();

			high_resolution_clock::time_point t2 = high_resolution_clock::now();
			measures[i] = duration_cast<microseconds>(t2 - t1).count();

			auto agents = sim->GetAgents();
			for (size_t i = 0; i < agentsCount; i++)
			{
				if (i > 0) myfile << ",";
				myfile << agents[i].posX << "," << agents[i].posY;
			}
			myfile << std::endl;
			std::cout << i << std::endl;
		}

		myfile.close();

		recording = sim->GetRecording();
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
