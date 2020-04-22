#include "pch.h"

#include "PinholeTestCase.h"
#include "TestCases/Utils.h"

using namespace FusionCrowd;

namespace TestFusionCrowd
{
	PinholeTestCase::PinholeTestCase(FusionCrowd::ComponentId opComponent, size_t agentsNum, size_t simulationSteps):
		ITestCase(agentsNum, simulationSteps),
		_opComponent(opComponent)
	{
	}

	void PinholeTestCase::Pre()
	{
		std::unique_ptr<ISimulatorBuilder, decltype(&BuilderDeleter)> builder(BuildSimulator(), BuilderDeleter);
		builder->WithNavMesh("Resources/pinhole.nav")
			->WithOp(_opComponent);

		_sim = std::unique_ptr<ISimulatorFacade, decltype(&SimulatorFacadeDeleter)>(builder->Build(), SimulatorFacadeDeleter);

		for (int i = 0; i < _agentsNum; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(2.0f, 5.0f), RandFloat(2.0f, 10.0f), _opComponent, ComponentIds::NAVMESH_ID, -1);
			_sim->SetAgentGoal(id, Point { RandFloat(35.0f, 37.0f), RandFloat(5.0f, 7.0f) });
		}
	}
}
