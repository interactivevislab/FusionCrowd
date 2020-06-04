#include "pch.h"

#include "TshapedFancyTestCase.h"
#include "TestCases/Utils.h"

using namespace FusionCrowd;

namespace TestFusionCrowd
{
	TshapedFancyTestCase::TshapedFancyTestCase(FusionCrowd::ComponentId opComponent, size_t agentsNum, size_t simulationSteps, bool writeTraj) :
		ITestCase(agentsNum, simulationSteps, writeTraj), _opComponent(opComponent)
	{
	}

	void TshapedFancyTestCase::Pre()
	{
		std::unique_ptr<ISimulatorBuilder, decltype(&BuilderDeleter)> builder(BuildSimulator(), BuilderDeleter);
		builder->WithNavMesh("Resources/t-shaped-fancy.nav")
			->WithOp(_opComponent);

		_sim = std::shared_ptr<ISimulatorFacade>(builder->Build(), SimulatorFacadeDeleter);

		size_t const half = _agentsNum / 2;
		size_t const otherhalf = _agentsNum - _agentsNum / 2;
		for (int i = 0; i < half; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(1.0f, 7.0f), RandFloat(1.0f, 3.0f), _opComponent, ComponentIds::NAVMESH_ID, -1);
			_sim->SetAgentGoal(id, Disk(RandFloat(26.5f, 29.5f), RandFloat(10.0f, 13.0f), 2.f));
		}

		for (int i = 0; i < otherhalf; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(1.0f, 4.0f), RandFloat(16.0f, 18.0f), _opComponent, ComponentIds::NAVMESH_ID, -1);
			_sim->SetAgentGoal(id, Disk(RandFloat(26.5f, 29.5f), RandFloat(10.0f, 13.0f), 2.f));
		}
	}
}
