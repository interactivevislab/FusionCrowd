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

		float x1 = -30, x2 = 30;
		float y1 = -30, y2 = 30;

		size_t group1 = _sim->AddGridFormation(x1, y1, 8, 0.5f);
		size_t group2 = _sim->AddGridFormation(x2, y2, 8, 0.5f);

		size_t dummy1 = _sim->GetGroupDummy(group1);
		size_t dummy2 = _sim->GetGroupDummy(group2);

		_sim->SetAgentGoal(dummy1, x2, y2);
		_sim->SetAgentGoal(dummy2, x1, y1);

		float z = sqrt(_agentsNum / 2.0f);

		for (size_t i = 0; i < _agentsNum / 2; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(x1 - z, y1), RandFloat(x1 - z / 2, y1 + z / 2), op, -1);

			_sim->SetAgentGroup(id, group1);
		}

		for (size_t i = _agentsNum / 2; i < _agentsNum; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(x2, y2 + z), RandFloat(x2 - z / 2, y2 + z / 2), op, -1);

			_sim->SetAgentGroup(id, group2);
		}
	}
}
