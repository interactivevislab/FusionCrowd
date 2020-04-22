#include "pch.h"

#include "GroupPerformanceTestCase.h"
#include "Export/Export.h"

using namespace FusionCrowd;

namespace TestFusionCrowd
{
	GroupPerformanceTestCase::GroupPerformanceTestCase() : ITestCase(8, 500, true)
	{
	}

	void GroupPerformanceTestCase::Pre()
	{
		std::unique_ptr<ISimulatorBuilder, decltype(&BuilderDeleter)> builder(BuildSimulator(), BuilderDeleter);
		builder
			->WithNavMesh("Resources/square.nav")
			->WithOp(ComponentIds::ORCA_ID);

		_sim = std::shared_ptr<ISimulatorFacade>(builder->Build(), SimulatorFacadeDeleter);

		// Fixed groups
		size_t g1 = _sim->AddGridGroup(-5.f, -4.75f, 2, 0.1f);
		size_t a1 = _sim->AddAgent(-5.f, -4.5f, ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::NO_COMPONENT);
		size_t a2 = _sim->AddAgent(-5.f,  -5.f, ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::NO_COMPONENT);
		_sim->AddAgentToGroup(a1, g1);
		_sim->AddAgentToGroup(a2, g1);

		size_t g2 = _sim->AddGridGroup(5.f, -4.5f, 2, 0.1f);
		size_t a3 = _sim->AddAgent(5.f, -4.75f, ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::NO_COMPONENT);
		size_t a4 = _sim->AddAgent(5.f, -4.25f, ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::NO_COMPONENT);
		_sim->AddAgentToGroup(a3, g2);
		_sim->AddAgentToGroup(a4, g2);

		_sim->SetGroupGoal(g1,  5.f, -4.75f);
		_sim->SetGroupGoal(g2, -5.f, -4.5f);

		// Just agents
		size_t a5 = _sim->AddAgent(-5.f, -2.5f, ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::NO_COMPONENT);
		_sim->SetAgentGoal(a5, Point { 5.f, -2.5f });

		size_t a6 = _sim->AddAgent(-5.f,  -3.f, ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::NO_COMPONENT);
		_sim->SetAgentGoal(a6, Point { 5.f, -3.f });


		size_t a7 = _sim->AddAgent(5.f, -2.75f, ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::NO_COMPONENT);
		_sim->SetAgentGoal(a7, Point { -5.f, -2.75f });

		size_t a8 = _sim->AddAgent(5.f,  -2.5f, ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::NO_COMPONENT);
		_sim->SetAgentGoal(a8, Point { -5.f, -2.5f });

		// Leader groups
		size_t a9  = _sim->AddAgent(-5.f, -0.5f, ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::NO_COMPONENT);
		size_t a10 = _sim->AddAgent(-5.f,  0.0f, ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::NO_COMPONENT);
		size_t g3 = _sim->AddGuidedGroup(a9);
		_sim->AddAgentToGroup(a10, g3);

		size_t a11 = _sim->AddAgent(5.f, -0.25f, ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::NO_COMPONENT);
		size_t a12 = _sim->AddAgent(5.f,  0.25f, ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::NO_COMPONENT);
		size_t g4 = _sim->AddGuidedGroup(a11);
		_sim->AddAgentToGroup(a12, g4);

		_sim->SetGroupGoal(g3,  5.f, -0.5f);
		_sim->SetGroupGoal(g4, -5.f, -0.25f);
	}
}
