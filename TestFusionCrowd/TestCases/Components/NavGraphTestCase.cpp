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
		_opComponent(FusionCrowd::ComponentIds::ORCA_ID)
	{
	}

	void NavGraphTestCase::Pre()
	{
		std::unique_ptr<ISimulatorBuilder, decltype(&BuilderDeleter)> builder(BuildSimulator(), BuilderDeleter);
		builder->WithNavMesh("Resources/square.nav")->
			WithNavGraph("Resources/graph/randomgrid_6x6.navgraph")->WithOp(_opComponent);


		_sim = std::unique_ptr<ISimulatorFacade, decltype(&SimulatorFacadeDeleter)>(builder->Build(), SimulatorFacadeDeleter);

		for (int i = 0; i < _agentsNum; i++)
		{
			float goalX = RandFloat(0, 60);
			float goalY = RandFloat(0, 60);
			float x = RandFloat(0, 60);
			float y = RandFloat(0, 60);

			size_t id;
			if (i % 2 == 0)
			{
				id = _sim->AddAgent(x, y, _opComponent, ComponentIds::NAVGRAPH_ID, -1);
			}
			else
			{
				id = _sim->AddAgent(x, y, _opComponent, ComponentIds::NAVMESH_ID, -1);
			}
			_sim->SetAgentGoal(id, Point {goalX, goalY});
		}
	}
}
