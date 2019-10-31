#include "pch.h"

#include "TshapedFancyTestCase.h"
#include <iostream>
#include <fstream>

#include "ThirdParty/date.h"
#include "Util/RecordingSerializer.h"

#include "TestCases/Utils.h"

using namespace FusionCrowd;
using namespace std::chrono;

namespace TestFusionCrowd
{
	TshapedFancyTestCase::TshapedFancyTestCase(FusionCrowd::ComponentId opComponent, size_t agentsNum, size_t simulationSteps, bool writeTrajectories):
		_opComponent(opComponent),
		_agentsNum(agentsNum),
		_simulationSteps(simulationSteps),
		_sim(nullptr, nullptr),
		_startTime(),
		_writeTrajectories(writeTrajectories)
	{
	}

	void TshapedFancyTestCase::Pre()
	{
		std::cout << "TshapedFancyTestCase test case setting up... ";
		std::unique_ptr<ISimulatorBuilder, decltype(&BuilderDeleter)> builder(BuildSimulator(), BuilderDeleter);
		builder->WithNavMesh("Resources/t-shaped-fancy.nav")
			->WithOp(_opComponent);

		_sim = std::unique_ptr<ISimulatorFacade, decltype(&SimulatorFacadeDeleter)>(builder->Build(), SimulatorFacadeDeleter);

		size_t const half = _agentsNum / 2;
		size_t const otherhalf = _agentsNum - _agentsNum / 2;
		for (int i = 0; i < half; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(1.0f, 7.0f), RandFloat(1.0f, 3.0f), _opComponent, -1);
			_sim->SetAgentGoal(id, RandFloat(26.5f, 29.5f), RandFloat(10.0f, 13.0f));
		}

		for (int i = 0; i < otherhalf; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(1.0f, 4.0f), RandFloat(16.0f, 18.0f), _opComponent, -1);
			_sim->SetAgentGoal(id, RandFloat(26.5f, 29.5f), RandFloat(10.0f, 13.0f));
		}

		std::cout << "done." << std::endl;
	}

	void TshapedFancyTestCase::Run()
	{
		std::cout << "Running TshapedFancyTestCase test case" << std::endl;

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

		std::cout << "TshapedFancyTestCase test case finished" << std::endl;
	}

	void TshapedFancyTestCase::Post()
	{
		std::cout << "TshapedFancyTestCase test case cleaning up... " << std::endl;

		std::string d = date::format("%Y%m%d_%H%M%S, ", _startTime);

		{ // RAII
			std::ofstream time_measures(d + "tshapedfancytestcase_step_times_microseconds.csv");
			std::cout << "Writing step times" << std::endl;

			for(auto val : _measures)
			{
				time_measures << val << std::endl;
			}
		}

		if(_writeTrajectories)
		{
			std::cout << "Writing trajectories" << std::endl;

			std::string filename(d + "tshapedfancytestcase_trajs.csv");
			auto & rec = _sim->GetRecording();
			Recordings::Serialize(rec, filename.c_str(), filename.size());
		}

		_sim = nullptr;
		std::cout << "Cleaned." << std::endl;
	}
}
