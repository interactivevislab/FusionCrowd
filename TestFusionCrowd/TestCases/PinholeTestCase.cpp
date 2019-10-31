#include "pch.h"

#include "PinholeTestCase.h"
#include <iostream>
#include <fstream>

#include "ThirdParty/date.h"
#include "Util/RecordingSerializer.h"
#include "TestCases/Utils.h"

#include <iostream>
#include <fstream>

using namespace FusionCrowd;
using namespace std::chrono;

namespace TestFusionCrowd
{
	PinholeTestCase::PinholeTestCase(FusionCrowd::ComponentId opComponent, size_t agentsNum, size_t simulationSteps, bool writeTrajectories):
		_opComponent(opComponent),
		_agentsNum(agentsNum),
		_simulationSteps(simulationSteps),
		_sim(nullptr, nullptr),
		_startTime(),
		_writeTrajectories(writeTrajectories)
	{
	}

	void PinholeTestCase::Pre()
	{
		std::cout << "Pinhole test case setting up... ";
		std::unique_ptr<ISimulatorBuilder, decltype(&BuilderDeleter)> builder(BuildSimulator(), BuilderDeleter);
		builder->WithNavMesh("Resources/pinhole.nav")
			->WithOp(_opComponent);

		_sim = std::unique_ptr<ISimulatorFacade, decltype(&SimulatorFacadeDeleter)>(builder->Build(), SimulatorFacadeDeleter);

		for (int i = 0; i < _agentsNum; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(2.0f, 5.0f), RandFloat(2.0f, 10.0f), _opComponent, -1);
			_sim->SetAgentGoal(id, RandFloat(35.0f, 37.0f), RandFloat(5.0f, 7.0f));
		}

		std::cout << "done." << std::endl;
	}

	void PinholeTestCase::Run()
	{
		std::cout << "Running pinhole test case" << std::endl;

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

		std::cout << "Pinhole test case finished" << std::endl;
	}

	void PinholeTestCase::Post()
	{
		std::cout << "Pinhole test case cleaning up... " << std::endl;

		std::string d = date::format("%Y%m%d_%H%M%S, ", _startTime);

		{ // RAII
			std::ofstream time_measures(d + "pinhole_step_times_microseconds.csv");
			std::cout << "Writing step times" << std::endl;

			for(auto val : _measures)
			{
				time_measures << val << std::endl;
			}
		}

		if(_writeTrajectories)
		{
			std::cout << "Writing trajectories" << std::endl;

			std::string filename(d + "pinhole_trajs.csv");
			auto & rec = _sim->GetRecording();
			Recordings::Serialize(rec, filename.c_str(), filename.size());
		}

		_sim = nullptr;
		std::cout << "Cleaned." << std::endl;
	}
}
