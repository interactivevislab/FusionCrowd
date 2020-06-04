#include "pch.h"

#include "NavGraphTestCase.h"
#include "TestCases/Utils.h"
#include <fstream>
#include <vector>
#include "Export/Fsm/IStrategyConfigurator.h"
#include "Export/Math/Shapes.h"

using namespace FusionCrowd;

namespace TestFusionCrowd
{
	NavGraphTestCase::NavGraphTestCase(size_t agentsNum, size_t simulationSteps, bool writeTraj) :
		ITestCase(agentsNum, simulationSteps, writeTraj),
		_opComponent(FusionCrowd::ComponentIds::TRANSPORT_ID)
	{
	}

	void NavGraphTestCase::Pre()
	{
		std::unique_ptr<ISimulatorBuilder, decltype(&BuilderDeleter)> builder(BuildSimulator(), BuilderDeleter);

		//builder->WithNavMesh("Resources/square.nav")->
		//builder->WithNavGraph("Resources/graph/randomgrid_6x6.navgraph")->WithOp(_opComponent);
		builder-> WithNavGraph("Resources/graph/crossroad.navgraph")->WithOp(_opComponent);

		_sim = std::unique_ptr<ISimulatorFacade, decltype(&SimulatorFacadeDeleter)>(builder->Build(), SimulatorFacadeDeleter);

		for (int i = 0; i < _agentsNum; i++)
		{


			float goalX, goalY, x, y;
			if (i % 4 == 0)
			{				
				x = RandFloat(80, 120);
				y = RandFloat(0, 30);
			}

			if (i % 4 == 1)
			{
				x = RandFloat(0, 30);
				y = RandFloat(80, 120);
			}

			if (i % 4 == 2)
			{
				x = RandFloat(170, 200);
				y = RandFloat(80, 120);
			}

			if (i % 4 == 3)
			{

				x = RandFloat(80, 120);
				y = RandFloat(170, 200);
			}

			int randInt = rand() % 4;
			if (randInt == 0)
			{
				goalX = 100;
				goalY = 200;
			}

			if (randInt == 1)
			{
				goalX = 200;
				goalY = 100;
			}

			if (randInt == 2)
			{
				goalX = 0;
				goalY = 100;
			}

			if (randInt == 3)
			{
				goalX = 100;
				goalY = 0;
			}

			/*float goalX = RandFloat(0, 60);
			float goalY = RandFloat(0, 60);
			float x = RandFloat(0, 60);
			float y = RandFloat(0, 60);*/

			size_t id;
			/*if (i % 2 == 0)
			{
				id = _sim->AddAgent(x, y, _opComponent, ComponentIds::NAVGRAPH_ID, -1);
			}
			else
			{
				id = _sim->AddAgent(x, y, _opComponent, ComponentIds::NAVMESH_ID, -1);
			}*/

			id = _sim->AddAgent(x, y, _opComponent, ComponentIds::NAVGRAPH_ID, -1);
			_sim->SetAgentGoal(id, Point {goalX, goalY});
		}
		_sim->AddTrafficLight(8);

	}
}
