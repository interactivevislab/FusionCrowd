#include "pch.h"
#include "NeighbourSearchBenchCase.h"

#include <iostream>
#include <fstream>
#include <random>
#include <memory>
#include <time.h>
#include <ctime>
#include <chrono>

#include "Math/consts.h"
#include "Benchmark/MicroscopicMetrics.h"
#include "Export/Export.h"

#include "TestCases/Utils.h"

namespace TestFusionCrowd
{
	using namespace DirectX::SimpleMath;
	using namespace std::chrono;
	using namespace FusionCrowd;

	NeighbourSearchBenchCase::NeighbourSearchBenchCase(float coef) : _coef(coef)
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

	void NeighbourSearchBenchCase::Run()
	{
		std::string navPath = "Resources/square.nav";

		std::shared_ptr<ISimulatorBuilder> builder(BuildSimulator(), BuilderDeleter);
		builder->WithNavMesh("Resources/square.nav")
			->WithOp(FusionCrowd::ComponentIds::KARAMOUZAS_ID)
			->WithOp(FusionCrowd::ComponentIds::ORCA_ID);

		std::shared_ptr<ISimulatorFacade> sim(builder->Build(), SimulatorFacadeDeleter);

		for (int i = 0; i < totalAgents; i++)
		{
			auto id = sim->AddAgent(RandFloat(0, worldSide), RandFloat(0, worldSide), FusionCrowd::ComponentIds::KARAMOUZAS_ID, -1);
			sim->SetAgentGoal(id, RandFloat(0, worldSide), RandFloat(0, worldSide));
		}

		std::ofstream myfile;
		myfile.open("way.csv");
		for (int i = 0; i < stepsTotal; i++)
		{
			high_resolution_clock::time_point t1 = high_resolution_clock::now();
			sim->DoStep();
			high_resolution_clock::time_point t2 = high_resolution_clock::now();
			measures[i] = duration_cast<microseconds>(t2 - t1).count();
		}

		myfile.close();

		recording = &(sim->GetRecording());
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
