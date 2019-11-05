#include "pch.h"
#include "StenkaNaStenkuTestCase.h"

#include "Export/Export.h"

#include "TestCases/Utils.h"

#include <memory>

using namespace FusionCrowd;

namespace TestFusionCrowd
{
	StenkaNaStenkuTestCase::StenkaNaStenkuTestCase(size_t agentsNum, size_t steps, bool writeTraj)
		:ITestCase(agentsNum, steps, writeTraj)
	{
	}

	void StenkaNaStenkuTestCase::Pre()
	{
		std::string navPath = "Resources/square.nav";

		ComponentId op = FusionCrowd::ComponentIds::ORCA_ID;

		std::shared_ptr<ISimulatorBuilder> builder(BuildSimulator(), BuilderDeleter);
		builder->WithNavMesh("Resources/square.nav")
			->WithOp(op);

		_sim = std::shared_ptr<FusionCrowd::ISimulatorFacade>(builder->Build(), SimulatorFacadeDeleter);
		_sim->SetIsRecording(WriteTrajectories);

		float z = sqrt(_agentsNum / 2.0f);

		for (size_t i = 0; i < _agentsNum / 2; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(500 - z, 500), RandFloat(500 -z / 2, 500 + z / 2), op, -1);
			_sim->SetAgentGoal(id, RandFloat(500.0f, 500 + z), RandFloat(500 -z / 2, 500 + z / 2));
		}

		for (size_t i = _agentsNum / 2; i < _agentsNum; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(500.0f, 500 + z), RandFloat(500 - z / 2, 500 + z / 2), op, -1);
			_sim->SetAgentGoal(id, RandFloat(500 - z, 500.0f), RandFloat(500 - z / 2, 500 + z / 2));
		}
	}
}
