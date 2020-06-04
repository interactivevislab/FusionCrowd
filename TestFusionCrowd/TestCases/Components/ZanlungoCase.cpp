#include "pch.h"
#include "ZanlungoCase.h"

#include "Math/consts.h"
#include "Export/Export.h"
#include "TestCases/Utils.h"

namespace TestFusionCrowd
{
	using namespace DirectX::SimpleMath;
	using namespace FusionCrowd;

	ZanlungoCase::ZanlungoCase() : ITestCase(100, 1000)
	{
	}

	void ZanlungoCase::Pre()
	{
		std::string navPath = "Resources/square.nav";

		std::shared_ptr<ISimulatorBuilder> builder(BuildSimulator(), BuilderDeleter);
		builder->WithNavMesh("Resources/square.nav")
			->WithOp(FusionCrowd::ComponentIds::ZANLUNGO_ID);

		_sim = std::shared_ptr<FusionCrowd::ISimulatorFacade>(builder->Build(), SimulatorFacadeDeleter);

		for (int i = 0; i < (totalAgents / 2 - 1); i++)
		{
			size_t id = _sim->AddAgent(RandFloat(2.0f, 4.0f), RandFloat(10.0f, 20.0f), ComponentIds::ZANLUNGO_ID, ComponentIds::NAVMESH_ID, -1);
			_sim->SetAgentGoal(id, Point { RandFloat(14.0f, 16.0f), RandFloat(10.0f, 20.0f) });
		}

		for (int i = (totalAgents / 2 - 1); i < totalAgents; i++)
		{
			size_t id = _sim->AddAgent(RandFloat(8.0f, 10.0f), RandFloat(20.0f, 25.0f), ComponentIds::ZANLUNGO_ID, ComponentIds::NAVMESH_ID, -1);
			_sim->SetAgentGoal(id, Point { RandFloat(8.0f, 10.0f), RandFloat(0.0f, 5.0f) });
		}
	}
}
