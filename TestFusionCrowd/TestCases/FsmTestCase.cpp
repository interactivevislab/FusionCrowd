#include "pch.h"

#include "FsmTestCase.h"

#include "ThirdParty/date.h"
#include "Util/RecordingSerializer.h"

#include "Export/Fsm/IStrategyConfigurator.h"
#include "Export/Fsm/IFsm.h"

#include "TestCases/Utils.h"

#include <iostream>
#include <fstream>

using namespace FusionCrowd;
using namespace std::chrono;

namespace TestFusionCrowd
{
	enum States : Fsm::State
	{
		Initial,
		HeadingTo1,
		HeadingTo2,
		HeadingTo3,
		HeadingTo4,
		Final
	};

	enum Events : Fsm::Event
	{
		Tick,
		Reached1,
		Reached2,
		Reached3,
		Reached4,
		TimerExpired
	};

	FsmTestCase::FsmTestCase(FusionCrowd::ComponentId opComponent, size_t agentsNum, size_t simulationSteps, bool writeTrajectories):
		_opComponent(opComponent),
		_agentsNum(agentsNum),
		_simulationSteps(simulationSteps),
		_sim(nullptr, nullptr),
		_startTime(),
		_writeTrajectories(writeTrajectories)
	{
	}

	void FsmTestCase::Pre()
	{
		std::cout << "Fsm test case setting up... ";

		std::unique_ptr<Fsm::IBuilder, decltype(&Fsm::BuilderDeleter)> fsmBuilder(Fsm::Builder(), Fsm::BuilderDeleter);

		auto * flowCW = fsmBuilder
			->WithStates()
				->Initial(States::Initial)
				->Intermediate(States::HeadingTo1)
				->Intermediate(States::HeadingTo2)
				->Intermediate(States::HeadingTo3)
				->Intermediate(States::HeadingTo4)
				->Final(States::Final)
			->WithTransitions()
				->Add(States::Initial,    States::HeadingTo1, Events::Tick)
				->Add(States::HeadingTo1, States::HeadingTo4, Events::Reached1)
				->Add(States::HeadingTo4, States::HeadingTo3, Events::Reached4)
				->Add(States::HeadingTo3, States::HeadingTo2, Events::Reached3)
				->Add(States::HeadingTo2, States::Final,      Events::Reached2)
			->Build();

		auto * flowCCW = fsmBuilder
			->WithStates()
				->Initial(States::Initial)
				->Intermediate(States::HeadingTo1)
				->Intermediate(States::HeadingTo2)
				->Intermediate(States::HeadingTo3)
				->Intermediate(States::HeadingTo4)
				->Final(States::Final)
			->WithTransitions()
				->Add(States::Initial,    States::HeadingTo2, Events::Tick)
				->Add(States::HeadingTo2, States::HeadingTo3, Events::Reached2)
				->Add(States::HeadingTo3, States::HeadingTo4, Events::Reached3)
				->Add(States::HeadingTo4, States::HeadingTo1, Events::Reached4)
				->Add(States::HeadingTo1, States::Final,      Events::Reached1)
			->Build();

		std::unique_ptr<ISimulatorBuilder, decltype(&BuilderDeleter)> builder(BuildSimulator(), BuilderDeleter);
		builder
			->WithNavMesh("Resources/t-shaped-fancy.nav")
			->WithOp(_opComponent)
		    ->WithStrategy(ComponentIds::FSM_ID);

		float const x1 = 4.0f; float const y1 = 2.0f;
		float const x2 = 3.0f; float const y2 = 17.0f;

		float const x3 = 28.0f; float const y3 = 6.5f;
		float const x4 = 28.0f; float const y4 = 12.0f;

		_sim = std::unique_ptr<ISimulatorFacade, decltype(&SimulatorFacadeDeleter)>(builder->Build(), SimulatorFacadeDeleter);
		_sim->SetIsRecording(_writeTrajectories);

		IStrategyComponent* tmp = _sim->GetStrategy(ComponentIds::FSM_ID);
		auto * fsmStrat = dynamic_cast<Fsm::IStrategyConfigurator *>(tmp);

		fsmStrat->CreateGoToAction(States::HeadingTo1, x1, y1);
		fsmStrat->CreateGoToAction(States::HeadingTo2, x2, y2);
		fsmStrat->CreateGoToAction(States::HeadingTo3, x3, y3);
		fsmStrat->CreateGoToAction(States::HeadingTo4, x4, y4);

		fsmStrat->SetTickEvent(Events::Tick);
		fsmStrat->CreateCloseToEvent(Events::Reached1, x1, y1);
		fsmStrat->CreateCloseToEvent(Events::Reached2, x2, y2);
		fsmStrat->CreateCloseToEvent(Events::Reached3, x3, y3);
		fsmStrat->CreateCloseToEvent(Events::Reached4, x4, y4);

		Fsm::AgentParams ccwMachine; ccwMachine.FsmId = fsmStrat->AddMachine(flowCCW);
		Fsm::AgentParams cwMachine;   cwMachine.FsmId = fsmStrat->AddMachine(flowCW);

		size_t firstHalf = _agentsNum / 2;
		for (int i = 0; i < firstHalf; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(x1 - 1, x1 + 1), RandFloat(y1 - 1, y1 + 1), _opComponent, ComponentIds::FSM_ID);
			_sim->SetAgentStrategyParam(id, ComponentIds::FSM_ID, cwMachine);
		}

		for (int i = firstHalf; i < _agentsNum; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(x2 - 1, x2 + 1), RandFloat(y2 - 1, y2 + 1), _opComponent, ComponentIds::FSM_ID);
			_sim->SetAgentStrategyParam(id, ComponentIds::FSM_ID, ccwMachine);
		}

		std::cout << "done." << std::endl;
	}

	void FsmTestCase::Run()
	{
		std::cout << "Running Fsm test case" << std::endl;

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

		std::cout << "Fsm test case finished" << std::endl;
	}

	void FsmTestCase::Post()
	{
		std::cout << "Fsm test case cleaning up... " << std::endl;

		std::string d = date::format("%Y%m%d_%H%M%S, ", _startTime);

		{ // RAII
			std::ofstream time_measures(d + "fsm_step_times_microseconds.csv");
			std::cout << "Writing step times" << std::endl;

			for(auto val : _measures)
			{
				time_measures << val << std::endl;
			}
		}

		if(_writeTrajectories)
		{
			std::cout << "Writing trajectories" << std::endl;

			std::string filename(d + "fsm_trajs.csv");
			auto & rec = _sim->GetRecording();
			Recordings::Serialize(rec, filename.c_str(), filename.size());
		}

		_sim = nullptr;
		std::cout << "Cleaned." << std::endl;
	}
}
