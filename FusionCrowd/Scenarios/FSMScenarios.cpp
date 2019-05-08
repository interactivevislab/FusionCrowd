#include "FSMScenarios.h"
#include "../OperationComponent/GCFComponent.h"
#include "../OperationComponent/HelbingComponent.h"
#include "../OperationComponent/KaramouzasComponent.h"
#include "../OperationComponent/ORCAComponent.h"
#include "../OperationComponent/PedVOComponent.h"
#include "../OperationComponent/ZanlungoComponent.h"

#include "../Navigation/SpatialQuery/NavMeshSpatialQuery.h"
#include "../Navigation/NavMesh/NavMeshLocalizer.h"

#include "../StrategyComponent/TestStrategyComponent.h"

#include "../StrategyComponent/FSM/FsmStrategy.h"
#include "../StrategyComponent/FSM/State.h"
#include "../StrategyComponent/FSM/Transition.h"
#include "../StrategyComponent/FSM/ConditionGoal.h"

#include <cstdlib>
#include <iostream>
#include <ctime>

namespace FusionCrowd
{
	namespace Scenarios
	{
		void FSMScenarios::RunSochiDemoScenarioInitialize(Simulator* sim, const char* pathNavMesh, int agentCount)
		{
#pragma region OperationComponent
			GCF::GCFComponent* gcfComponent = new GCF::GCFComponent();
			Helbing::HelbingComponent* helbingComponent = new Helbing::HelbingComponent();
			Karamouzas::KaramouzasComponent* karamouzasComponent = new Karamouzas::KaramouzasComponent();
			ORCA::ORCAComponent* orcaComponent = new ORCA::ORCAComponent();
			PedVO::PedVOComponent* pedVOComponent = new PedVO::PedVOComponent();
			Zanlungo::ZanlungoComponent* zComponent = new Zanlungo::ZanlungoComponent();

			sim->AddOperComponent(gcfComponent);
			sim->AddOperComponent(helbingComponent);
			sim->AddOperComponent(karamouzasComponent);
			sim->AddOperComponent(orcaComponent);
			sim->AddOperComponent(pedVOComponent);
			sim->AddOperComponent(zComponent);
#pragma endregion OperationComponent

#pragma  region FSM
			FusionCrowd::Strategy::FsmStrategy* fsm = new FusionCrowd::Strategy::FsmStrategy();

			Goal* goal = new FusionCrowd::PointGoal(-3.0f, 5.0f);

			FusionCrowd::Strategy::FSM::State* sW1 = new FusionCrowd::Strategy::FSM::State("Walk1", false, goal);
			FusionCrowd::Strategy::FSM::State* sF1 = new FusionCrowd::Strategy::FSM::State("Stop1", true, goal);
			fsm->AddState(sW1);
			fsm->AddState(sF1);

			FusionCrowd::Strategy::FSM::ConditionGoal* con = new FusionCrowd::Strategy::FSM::ConditionGoal(0.1f);
			FusionCrowd::Strategy::FSM::Transition* transition = new FusionCrowd::Strategy::FSM::Transition("Walk1", "Stop1", con);
			fsm->AddTransitions(transition);
#pragma endregion FSMEnd

			std::vector<DirectX::SimpleMath::Vector2> posVector = GetPositionAgent(agentCount);

			//fsm->AddAgent(&sim.agents[0], "Walk1");

			bool r = fsm->BuildFSM();
		}

		void FSMScenarios::AddSpatialQuery(Simulator* sim, FusionCrowd::SpatialQuery* spatialQuery)
		{
			sim->AddSpatialQuery(spatialQuery);
		}

		std::vector<DirectX::SimpleMath::Vector2> FSMScenarios::GetPositionAgent(int agentCount)
		{
			std::vector<DirectX::SimpleMath::Vector2> pos;
			float startX = -100;
			float startY = 104;
			for (int i = 0; i < agentCount; i++)
			{
				pos.push_back(DirectX::SimpleMath::Vector2(startX, startY));
				startX = startX - 0.5f;
				if (i % 10 == 0)
				{
					startX = -100;
					startY = startY + 1;
				}
			}
			return pos;
		}
	}
}