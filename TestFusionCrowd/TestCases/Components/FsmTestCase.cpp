#include "pch.h"

#include "FsmTestCase.h"

#include "Export/Export.h"
#include "Export/Fsm/IStrategyConfigurator.h"
#include "Export/Fsm/IFsm.h"

#include "TestCases/Utils.h"

using namespace FusionCrowd;

namespace TestFusionCrowd
{
	enum States : Fsm::State
	{
		Initial,
		Deciding,
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
		ITestCase(agentsNum, simulationSteps, writeTrajectories),
		_opComponent(opComponent)
	{
	}

	void FsmTestCase::Pre()
	{
		std::unique_ptr<Fsm::IBuilder, decltype(&Fsm::BuilderDeleter)> fsmBuilder(Fsm::Builder(), Fsm::BuilderDeleter);

		FCArray<Fsm::State> start(2);
		start[0] = States::HeadingTo3;
		start[1] = States::HeadingTo4;

		FCArray<Fsm::State> heading1234(4);
		heading1234[0] = States::HeadingTo1;
		heading1234[1] = States::HeadingTo2;
		heading1234[2] = States::HeadingTo3;
		heading1234[3] = States::HeadingTo4;

		auto * flow = fsmBuilder
			->WithStates()
				->Initial(States::Initial)
				->Intermediate(States::Deciding)
				->Intermediate(States::HeadingTo1)
				->Intermediate(States::HeadingTo2)
				->Intermediate(States::HeadingTo3)
				->Intermediate(States::HeadingTo4)
				->Final(States::Final)
			->WithTransitions()
				->Add(States::Initial, States::Deciding, Events::Tick)
				->AddRandom(States::Deciding, start, Events::TimerExpired)
				->AddRandom(States::HeadingTo3, heading1234, Events::Reached3)
				->AddRandom(States::HeadingTo4, heading1234, Events::Reached4)
				->Add(States::HeadingTo1, States::Final, Events::Reached1)
				->Add(States::HeadingTo2, States::Final, Events::Reached2)
			->Build();

		std::unique_ptr<ISimulatorBuilder, decltype(&BuilderDeleter)> builder(BuildSimulator(), BuilderDeleter);
		builder
			->WithNavMesh("Resources/t-shaped-fancy.nav")
			->WithOp(_opComponent)
		    ->WithStrategy(ComponentIds::FSM_ID);

		const Fsm::Point p1 = {4.0f, 2.0f};
		const Fsm::Point p2 = {3.0f, 17.0f};
		const Fsm::Point p3 = {28.0f, 6.5f};
		const Fsm::Point p4 = {28.0f, 12.0f};

		_sim = std::unique_ptr<ISimulatorFacade, decltype(&SimulatorFacadeDeleter)>(builder->Build(), SimulatorFacadeDeleter);
		_sim->SetIsRecording(WriteTrajectories);

		IStrategyComponent* tmp = _sim->GetStrategy(ComponentIds::FSM_ID);
		auto * fsmStrat = dynamic_cast<Fsm::IStrategyConfigurator *>(tmp);

		size_t fsmId = fsmStrat->AddMachine(flow);

		fsmStrat->CreateGoToAction(fsmId, States::HeadingTo1, p1);
		fsmStrat->CreateGoToAction(fsmId, States::HeadingTo2, p2);
		fsmStrat->CreateGoToAction(fsmId, States::HeadingTo3, p3);
		fsmStrat->CreateGoToAction(fsmId, States::HeadingTo4, p4);

		fsmStrat->SetTickEvent(fsmId, Events::Tick);
		fsmStrat->CreateTimerEvent(fsmId, States::Deciding, Events::TimerExpired, 0.0f, 100.0f);
		fsmStrat->CreatePointReachEvent(fsmId, Events::Reached1, p1, 2.0f);
		fsmStrat->CreatePointReachEvent(fsmId, Events::Reached2, p2, 2.0f);
		fsmStrat->CreatePointReachEvent(fsmId, Events::Reached3, p3, 2.0f);
		fsmStrat->CreatePointReachEvent(fsmId, Events::Reached4, p4, 2.0f);

		Fsm::AgentParams flowMachineParams; flowMachineParams.FsmId = fsmId;

		size_t firstHalf = _agentsNum / 2;
		for (int i = 0; i < firstHalf; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(p1.x - 1, p1.x + 1), RandFloat(p1.y - 1, p1.y + 1), _opComponent, ComponentIds::NAVMESH_ID, ComponentIds::FSM_ID);
			_sim->SetAgentStrategyParam(id, ComponentIds::FSM_ID, flowMachineParams);
		}

		for (int i = firstHalf; i < _agentsNum; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(p2.x - 1, p2.x + 1), RandFloat(p2.y - 1, p2.y + 1), _opComponent, ComponentIds::NAVMESH_ID, ComponentIds::FSM_ID);
			_sim->SetAgentStrategyParam(id, ComponentIds::FSM_ID, flowMachineParams);
		}
	}
}
