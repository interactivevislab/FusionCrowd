#include "pch.h"

#include "CrossingTestCase.h"

#include "TestCases/Utils.h"

using namespace FusionCrowd;

namespace TestFusionCrowd
{
	CrossingTestCase::CrossingTestCase(FusionCrowd::ComponentId opComponent, size_t agentsNum, size_t simulationSteps, bool writeTrajectories):
		ITestCase(agentsNum, simulationSteps, writeTrajectories),
		_opComponent(opComponent)
	{
	}

	void CrossingTestCase::Pre()
	{
		std::unique_ptr<ISimulatorBuilder, decltype(&BuilderDeleter)> builder(BuildSimulator(), BuilderDeleter);
		builder->WithNavMesh("Resources/crossing.nav")
			->WithOp(_opComponent);

		_sim = std::unique_ptr<ISimulatorFacade, decltype(&SimulatorFacadeDeleter)>(builder->Build(), SimulatorFacadeDeleter);

		for (int i = 0; i < (_agentsNum / 2 - 1); i++)
		{
			size_t id = _sim->AddAgent(RandFloat(80.0f, 120.0f), RandFloat(0.0f, 40.0f), _opComponent, ComponentIds::NAVMESH_ID, -1);
			_sim->SetAgentGoal(id, Point { RandFloat(80.0f, 120.0f), RandFloat(160.0f, 200.0f) });
		}

		for (int i = (_agentsNum / 2 - 1); i < _agentsNum; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(0.0f, 40.0f), RandFloat(80.0f, 120.0f), _opComponent, ComponentIds::NAVMESH_ID, -1);
			_sim->SetAgentGoal(id, Point { RandFloat(160.0f, 200.0f), RandFloat(80.0f, 120.0f) });
		}
	}
}
