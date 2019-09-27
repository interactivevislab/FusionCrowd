#include "pch.h"

#include "CrossingTestCase.h"
#include <iostream>
#include <fstream>

#include "ThirdParty/date.h"

using namespace FusionCrowd;
using namespace std::chrono;

namespace TestFusionCrowd
{
	CrossingTestCase::CrossingTestCase(FusionCrowd::ComponentId opComponent, size_t agentsNum, size_t simulationSteps, bool writeTrajectories):
		_opComponent(opComponent),
		_agentsNum(agentsNum),
		_simulationSteps(simulationSteps),
		_sim(nullptr, nullptr),
		_startTime(),
		_writeTrajectories(writeTrajectories)
	{
	}

	void CrossingTestCase::Pre()
	{
		std::cout << "Corridor test case setting up... ";
		std::unique_ptr<ISimulatorBuilder, decltype(&BuilderDeleter)> builder(BuildSimulator(), BuilderDeleter);
		builder->WithNavMesh("Resources/crossing.nav")
			->WithOp(_opComponent);

		_sim = std::unique_ptr<ISimulatorFacade, decltype(&SimulatorFacadeDeleter)>(builder->Build(), SimulatorFacadeDeleter);

		for (int i = 0; i < (_agentsNum / 2 - 1); i++)
		{
			size_t id = _sim->AddAgent(RandFloat(80.0f, 120.0f), RandFloat(0.0f, 40.0f), _opComponent, -1);
			_sim->SetAgentGoal(id, RandFloat(80.0f, 120.0f), RandFloat(160.0f, 200.0f));
		}

		for (int i = (_agentsNum / 2 - 1); i < _agentsNum; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(0.0f, 40.0f), RandFloat(80.0f, 120.0f), _opComponent, -1);
			_sim->SetAgentGoal(id, RandFloat(160.0f, 200.0f), RandFloat(80.0f, 12.0f));
		}

		std::cout << "done." << std::endl;
	}

	void CrossingTestCase::Run(const float& args)
	{
		std::cout << "Running Corridor test case" << std::endl;

		_startTime = system_clock::now();
		for (int i = 0; i < _simulationSteps; i++)
		{
			high_resolution_clock::time_point t1 = high_resolution_clock::now();
			_sim->DoStep();

			high_resolution_clock::time_point t2 = high_resolution_clock::now();
			long long duration = duration_cast<microseconds>(t2 - t1).count();
			_measures.push_back(duration);

			if(i % 10 == 0)
			{
				std::cout << "  Simulation step "
						  << i << "/" << _simulationSteps
			              << " in " << duration << " microseconds"
			              << std::endl;
			}
		}

		std::cout << "Corridor test case finished" << std::endl;
	}

	void CrossingTestCase::Post()
	{
		std::cout << "Corridor test case cleaning up... " << std::endl;

		std::string d = date::format("%Y%m%d_%H%M%S, ", _startTime);

		{ // RAII
			std::ofstream time_measures(d + "crossing_step_times_microseconds.csv");
			std::cout << "Writing step times" << std::endl;

			for(auto val : _measures)
			{
				time_measures << val << std::endl;
			}
		}

		if(_writeTrajectories)
		{
			std::cout << "Writing trajectories" << std::endl;
			std::ofstream trajs(d + "crossing_trajs.csv");

			auto rec = _sim->GetRecording();

			auto timespan = rec->GetTimeSpan();
			for(float step : timespan)
			{
				auto & slice = rec->GetSlice(step);

				auto ids = slice.GetAgentIds();
				bool first = true;
				for(size_t id : ids)
				{
					auto info = slice.GetAgentInfo(id);
					if(!first)
					{
						trajs << ", ";
					}

					trajs << info.posX << ", " << info.posY;
					first = false;
				}
				trajs << std::endl;
			}
		}

		_sim = nullptr;
		std::cout << "Cleaned." << std::endl;
	}

	float CrossingTestCase::RandFloat(float min, float max)
	{
		return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min)));
	}

}