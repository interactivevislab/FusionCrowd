#include "pch.h"
#include "GroupMovementTestCase.h"

#include "Export/Export.h"
#include "Export/Fsm/IStrategyConfigurator.h"
#include "Export/Fsm/IFsm.h"

#include "TestCases/Utils.h"

#include <memory>
#include <array>

using namespace FusionCrowd;

namespace TestFusionCrowd
{
	GroupMovementTestCase::GroupMovementTestCase(size_t steps, bool writeTraj)
		: ITestCase(10, steps, writeTraj)
	{
	}

	enum States : Fsm::State
	{
		Initial    = 10000,
		HeadingTo0 = 0,
		HeadingTo1 = 1,
		HeadingTo2 = 2,
		HeadingTo3 = 3,
		HeadingTo4 = 4,
		Final      = 9999
	};

	enum Events : Fsm::Event
	{
		Reached0 = 0,
		Reached1 = 1,
		Reached2 = 2,
		Reached3 = 3,
		Reached4 = 4
	};

	void GroupMovementTestCase::Pre()
	{
		std::unique_ptr<Fsm::IBuilder, decltype(&Fsm::BuilderDeleter)> fsmBuilder(Fsm::Builder(), Fsm::BuilderDeleter);

		FCArray<Fsm::State> cycle(2);
		cycle[0] = States::HeadingTo1;
		cycle[1] = States::HeadingTo3;

		auto * flow = fsmBuilder
			->WithStates()
				->Initial(States::Initial)
				->Intermediate(States::HeadingTo0)
				->Intermediate(States::HeadingTo1)
				->Intermediate(States::HeadingTo2)
				->Intermediate(States::HeadingTo3)
				->Intermediate(States::HeadingTo4)
				->Final(States::Final)
			->WithTransitions()
				->Add(States::Initial, States::HeadingTo0, Events::Reached0)
				->AddRandom(States::HeadingTo0, cycle, Events::Reached0)
				->Add(States::HeadingTo1, States::HeadingTo2, Events::Reached1)
				->Add(States::HeadingTo2, States::HeadingTo0, Events::Reached2)
				->Add(States::HeadingTo3, States::HeadingTo4, Events::Reached3)
				->Add(States::HeadingTo4, States::HeadingTo0, Events::Reached4)
			->Build();

		std::unique_ptr<ISimulatorBuilder, decltype(&BuilderDeleter)> builder(BuildSimulator(), BuilderDeleter);
		builder
			->WithNavGraph("Resources/graph/twocycles.navgraph")
			->WithOp(ComponentIds::ORCA_ID)
		    ->WithStrategy(ComponentIds::FSM_ID);

		std::array<Fsm::Point, 5> p = {{
			{  0.f,  0.f},
			{-10.f, -5.f},
			{-10.f,  5.f},
			{ 10.f, -5.f},
			{ 10.f,  5.f}
		}};

		_sim = std::unique_ptr<ISimulatorFacade, decltype(&SimulatorFacadeDeleter)>(builder->Build(), SimulatorFacadeDeleter);
		_sim->SetIsRecording(WriteTrajectories);

		IStrategyComponent* tmp = _sim->GetStrategy(ComponentIds::FSM_ID);
		auto * fsmStrat = dynamic_cast<Fsm::IStrategyConfigurator *>(tmp);

		size_t fsmId = fsmStrat->AddMachine(flow);

		fsmStrat->CreateGoToAction(fsmId, States::HeadingTo0, p[0]);
		fsmStrat->CreateGoToAction(fsmId, States::HeadingTo1, p[1]);
		fsmStrat->CreateGoToAction(fsmId, States::HeadingTo2, p[2]);
		fsmStrat->CreateGoToAction(fsmId, States::HeadingTo3, p[3]);
		fsmStrat->CreateGoToAction(fsmId, States::HeadingTo4, p[4]);

		float reachRadius = 1.5f;

		fsmStrat->CreatePointReachEvent(fsmId, Events::Reached0, p[0], reachRadius);
		fsmStrat->CreatePointReachEvent(fsmId, Events::Reached1, p[1], reachRadius);
		fsmStrat->CreatePointReachEvent(fsmId, Events::Reached2, p[2], reachRadius);
		fsmStrat->CreatePointReachEvent(fsmId, Events::Reached3, p[3], reachRadius);
		fsmStrat->CreatePointReachEvent(fsmId, Events::Reached4, p[4], reachRadius);

		Fsm::AgentParams flowMachineParams; flowMachineParams.FsmId = fsmId;


		size_t leaderId = _sim->AddAgent(RandFloat(-2, 2), RandFloat(-2, 2), ComponentIds::ORCA_ID, ComponentIds::NAVGRAPH_ID, ComponentIds::NO_COMPONENT);
		size_t groupId = _sim->AddGuidedGroup(leaderId);
		size_t groupDummyId = _sim->GetGroupDummyAgent(groupId);

		_sim->SetAgentOp(groupDummyId, ComponentIds::ORCA_ID);
		_sim->SetAgentTactic(groupDummyId, ComponentIds::NAVGRAPH_ID);
		_sim->SetAgentStrategy(groupDummyId, ComponentIds::FSM_ID);

		for (int i = 1; i < GetAgentCount(); i++)
		{
			size_t id = _sim->AddAgent(RandFloat(-2, 2), RandFloat(-2, 2), ComponentIds::ORCA_ID, ComponentIds::NAVGRAPH_ID, ComponentIds::NO_COMPONENT);
			_sim->AddAgentToGroup(id, groupId);
		}

		size_t groupDummy = _sim->GetGroupDummyAgent(groupId);

		_sim->SetAgentStrategyParam(groupDummy, ComponentIds::FSM_ID, flowMachineParams);
	}
}
