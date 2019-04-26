#include "DemoScenarios.h"

#include "../OperationComponent/GCFComponent.h"
#include "../OperationComponent/HelbingComponent.h"
#include "../OperationComponent/KaramouzasComponent.h"
#include "../OperationComponent/ORCAComponent.h"
#include "../OperationComponent/PedVOComponent.h"
#include "../OperationComponent/ZanlungoComponent.h"

#include "../Navigation/SpatialQuery/NavMeshSpatialQuery.h"
#include "../Navigation/NavMesh/NavMeshLocalizer.h"

#include "../StrategyComponent/TestStrategyComponent.h"

#include <cstdlib>
#include <iostream>
#include <ctime>

namespace FusionCrowd
{
	namespace Scenarios
	{
		void DemoScenarios::RunSochiDemoScenarioInitialize(Simulator* sim, const char* pathNavMesh, int agentCount)
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

			FusionCrowd::NavMeshSpatialQuery* sq = new FusionCrowd::NavMeshSpatialQuery();

			NavMeshComponent nav;
			nav._localizer = loadNavMeshLocalizer(pathNavMesh, true);
			sq->SetNavMeshLocalizer(nav._localizer);

			sim->AddSpatialQuery(sq);
			sim->nav = nav;


			FusionCrowd::Strategy::TestStrategyComponent* stTest = new FusionCrowd::Strategy::TestStrategyComponent();
			Goal* goal = new FusionCrowd::PointGoal(30, -57);
			goal->setID(0);

			sim->AddStrategyComponent(stTest);

			std::srand(unsigned(std::time(0)));
			std::vector<DirectX::SimpleMath::Vector2> posVector = GetPositionAgent(agentCount);

			for (int i = 0; i < agentCount; i++)
			{
				DirectX::SimpleMath::Vector2 pos(0, 0);
				sim->AddAgent(360, 10, 1, 5, 0.19f, 0.05f, 0.2f, 5, posVector[i]);
				int id = sim->agents.size() - 1;
				int randIndex = (std::rand() % static_cast<int>(sim->operComponents.size() - 4));
				switch(3)
				{
				case 0:
				{
					gcfComponent->AddAgent(&sim->agents[id], 80);
					stTest->AddGoal(id, goal);
					sim->agents[id]._operationComponent = 0;
					break;
				}
				case 1:
				{
					helbingComponent->AddAgent(id, 80);
					stTest->AddGoal(id, goal);
					sim->agents[id]._operationComponent = 1;
					break;
				}
				case 2:
				{
					karamouzasComponent->AddAgent(id, 0.69f, 8.f);
					stTest->AddGoal(id, goal);
					sim->agents[id]._operationComponent = 2;
					break;
				}
				case 3:
				{
					orcaComponent->AddAgent(id);
					stTest->AddGoal(id, goal);
					sim->agents[id]._operationComponent = 3;
					break;
				}
				case 4:
				{
					pedVOComponent->AddAgent(id, 2.5f, 0.15f, 1.0f, true, 1.57f, 0.5f);
					stTest->AddGoal(id, goal);
					sim->agents[id]._operationComponent = 4;
					break;
				}
				case 5:
				{
					zComponent->AddAgent(id, 80);
					stTest->AddGoal(id, goal);
					sim->agents[id]._operationComponent = 5;
					break;
				}
				default:
					break;
				}
				sim->nav._localizer->updateLocation(&sim->agents[id]);
				goal->assign(&sim->agents[id]);
			}

			std::vector<FusionCrowd::Agent * > agtPointers(sim->agents.size());
			for (size_t a = 0; a < sim->agents.size(); ++a) {
				agtPointers[a] = &sim->agents[a];
			}

			sim->spatialQuerys[0]->SetAgents(agtPointers);
			sim->spatialQuerys[0]->ProcessObstacles();
		}

		void DemoScenarios::AddSpatialQuery(Simulator* sim, FusionCrowd::SpatialQuery* spatialQuery)
		{
			sim->AddSpatialQuery(spatialQuery);
		}
		std::vector<DirectX::SimpleMath::Vector2> DemoScenarios::GetPositionAgent(int agentCount)
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
					startY = startY +1;
				}
			}
			return pos;
		}
	}
}
