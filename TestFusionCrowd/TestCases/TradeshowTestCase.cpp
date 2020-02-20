#include "pch.h"
#include "TradeshowTestCase.h"

#include "TestCases/Utils.h"

#include "Export/Export.h"
#include "Export/ComponentId.h"
#include "Export/Fsm/IStrategyConfigurator.h"
#include "Export/Fsm/IFsm.h"

#include <fstream>
#include <vector>

using namespace FusionCrowd;

namespace TestFusionCrowd
{
	enum States : Fsm::State
	{
		Initial,
		Walk,
		Arrive,
		DecideFeeling,
		Happy,
		Angry,
		ChooseNext
	};

	enum Events : Fsm::Event
	{
		Tick,
		ReachedStand,
		FeelingTimer,
		AngryTimer,
		HappyTimer
	};

	void TradeshowTestCase::Pre()
	{
		std::unique_ptr<ISimulatorBuilder, decltype(&BuilderDeleter)> builder(BuildSimulator(), BuilderDeleter);
		builder->WithNavMesh("Resources/tradeshow.nav")
			->WithOp(_op)
			->WithStrategy(ComponentIds::FSM_ID);

		_sim = std::shared_ptr<ISimulatorFacade>(builder->Build(), SimulatorFacadeDeleter);

		std::unique_ptr<Fsm::IBuilder, decltype(&Fsm::BuilderDeleter)> fsmBuilder(Fsm::Builder(), Fsm::BuilderDeleter);

		FCArray<Fsm::State> decisions(2);
		decisions[0] = States::Angry;
		decisions[1] = States::Happy;

		auto * fsm = fsmBuilder
			->WithStates()
				->Initial(States::Initial)
				->Intermediate(States::ChooseNext)
				->Intermediate(States::Walk)
				->Intermediate(States::Arrive)
				->Intermediate(States::DecideFeeling)
				->Intermediate(States::Angry)
				->Intermediate(States::Happy)
			->WithTransitions()
				->Add(States::Initial, States::ChooseNext, Events::Tick)
				->Add(States::ChooseNext, States::Walk, Events::Tick)
				->Add(States::Walk, States::Arrive, Events::ReachedStand)
				->Add(States::Arrive, States::DecideFeeling, Events::FeelingTimer)
				->AddRandom(States::DecideFeeling, decisions, Events::Tick)
				->Add(States::Angry, States::ChooseNext, Events::AngryTimer)
				->Add(States::Happy, States::ChooseNext, Events::HappyTimer)
			->Build();

		auto * fsmStrat = dynamic_cast<Fsm::IStrategyConfigurator *>(_sim->GetStrategy(ComponentIds::FSM_ID));

		size_t fsmId = fsmStrat->AddMachine(fsm);

		std::ifstream tradeshowGoals("Resources/TradeshowGoals.txt");
		std::vector<Fsm::Point> vectorGoals;
		float x = 0;
		float y = 0;
		while(tradeshowGoals >> x >> y)
		{
			vectorGoals.push_back({x, y});
		}

		FCArray<Fsm::Point> goals(vectorGoals.size());
		std::copy(vectorGoals.begin(), vectorGoals.end(), goals.begin());

		fsmStrat->CreateGoToAnyAction(fsmId, States::ChooseNext, goals);
		fsmStrat->SetTickEvent(fsmId, Events::Tick);
		fsmStrat->CreateTimerEvent(fsmId, States::Arrive, Events::FeelingTimer, 2.0f, 4.0f);
		fsmStrat->CreateTimerEvent(fsmId, States::Angry, Events::AngryTimer, 1.0f, 2.0f);
		fsmStrat->CreateTimerEvent(fsmId, States::Happy, Events::HappyTimer, 4.0f, 6.0f);
		fsmStrat->CreateAnyPointReachEvent(fsmId, Events::ReachedStand, goals);


		Fsm::AgentParams flowMachineParams; flowMachineParams.FsmId = fsmId;

		std::ifstream agentPositions("Resources/TradeshowAgents.txt");
		while(agentPositions >> x >> y)
		{
			size_t id = _sim->AddAgent(x, y, _op, ComponentIds::NAVMESH_ID, ComponentIds::FSM_ID);
			_sim->SetAgentStrategyParam(id, ComponentIds::FSM_ID, flowMachineParams);
		}
	}
}
